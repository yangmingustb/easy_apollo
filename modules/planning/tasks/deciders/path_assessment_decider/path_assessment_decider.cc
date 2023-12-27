/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/tasks/deciders/path_assessment_decider/path_assessment_decider.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"
#include "modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.h"
#include "modules/common/collision_detection/gjk2d_interface.h"
#include "modules/common/math/polygon_base.h"
#include "modules/planning/constraint_checker/collision_checker.h"

namespace apollo
{
namespace planning
{
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

#define debug_path_assessment_decider_time (0)

namespace
{
// PointDecision contains (s, PathPointType, distance to closest obstacle).
using PathPointDecision = std::tuple<double, PathData::PathPointType, double>;
constexpr double kMinObstacleArea = 1e-4;
}  // namespace

PathAssessmentDecider::PathAssessmentDecider(
        const TaskConfig& config,
        const std::shared_ptr<DependencyInjector>& injector) :
    Decider(config, injector)
{
}

Status PathAssessmentDecider::Process(
        Frame* const frame, ReferenceLineInfo* const reference_line_info)
{
    // Sanity checks.
    CHECK_NOTNULL(frame);
    CHECK_NOTNULL(reference_line_info);

    // skip path_assessment_decider if reused path
    if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable())
    {
        AINFO << "path reuse";
        return Status::OK();
    }

    const auto& candidate_path_data =
            reference_line_info->GetCandidatePathData();

    const auto& end_time0 = std::chrono::system_clock::now();

#if debug_path_assessment_decider

    if (candidate_path_data.empty())
    {
        AINFO << "Candidate path data is empty.";
    }
    else
    {
        AINFO << "There are " << candidate_path_data.size()
               << " candidate paths";
    }

    for (const auto& path_bound :
         reference_line_info->GetCandidatePathBoundaries())
    {
        AINFO << "path bound label " << path_bound.label() << "\n";
    }

#endif

    // 1. Remove invalid path. 根据偏移量过滤
    // check 越过路网，和static obstacle碰撞情况
    std::vector<PathData> valid_path_data;
    for (const auto& curr_path_data : candidate_path_data)
    {
#if debug_path_assessment_decider
        AINFO << "path label " << curr_path_data.path_label();
#endif

        // RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
        //                 reference_line_info);

        if (curr_path_data.path_label().find("fallback") != std::string::npos)
        {
            if (IsValidFallbackPath(*reference_line_info, curr_path_data))
            {
                valid_path_data.push_back(curr_path_data);
            }
        }
        else
        {
            if (IsValidRegularPath(*reference_line_info, curr_path_data))
            {
                valid_path_data.push_back(curr_path_data);
            }
        }
    }

#if debug_path_assessment_decider

    if (valid_path_data.size() < 1)
    {
        AINFO << "Candidate path data is empty.";
    }
    else
    {
        AINFO << "There are " << valid_path_data.size() << " candidate paths";
    }

    for (const auto& path : valid_path_data)
    {
        AINFO << "path bound label " << path.path_label();
    }

#endif

    const auto& end_time1 = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_time1 - end_time0;

#if debug_path_assessment_decider_time
    AINFO << "Time for path validity checking: " << diff.count() * 1000
          << " msec.";
#endif

    // 2. Analyze and add important info for speed decider to use
    // check 压线情况
    size_t valid_path_cnt = 0;
    const Obstacle* blocking_obstacle_on_selflane = nullptr;

    for (size_t i = 0; i != valid_path_data.size(); ++i)
    {
        auto& curr_path_data = valid_path_data[i];

        // fallback path， valid
        if (curr_path_data.path_label().find("fallback") != std::string::npos)
        {
            // remove empty path_data.
            if (!curr_path_data.Empty())
            {
                if (valid_path_cnt != i)
                {
                    valid_path_data[valid_path_cnt] = curr_path_data;
                }
                ++valid_path_cnt;
            }
            continue;
        }

#if debug_path_assessment_decider_time
        const auto& set_path_info_start = std::chrono::system_clock::now();
#endif
        // 计算path是否越过车道
        SetPathInfo(*reference_line_info, &curr_path_data);

#if debug_path_assessment_decider_time
        const auto& set_path_info_end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = set_path_info_end - set_path_info_start;
        AINFO << "Time for path cross lane checking: " << diff.count() * 1000
              << " msec.";
#endif

        // Trim all the lane-borrowing paths so that it ends with an in-lane
        // position.
        // 对于lane borrow path, 如果当前车道停了20辆车，可能lane borrow
        // path 的所有点都是out lane,
        // so经过trim后整条路径的长度变成0或很小，导致无法lane borrow。
        // 目前认为这是合理现象。
        // 感觉设计逻辑是：如果lane borrow不能返回车道，那么该path valid data就很短，
        // 导致不lane borrow
        if (curr_path_data.path_label().find("pullover") == std::string::npos)
        {
            TrimTailingOutLanePoints(&curr_path_data);
        }

        // find blocking_obstacle_on_selflane, to be used for lane selection
        // later. lane keep path
        if (curr_path_data.path_label().find("self") != std::string::npos)
        {
            const auto blocking_obstacle_id =
                    curr_path_data.blocking_obstacle_id();
            blocking_obstacle_on_selflane =
                    reference_line_info->path_decision()->Find(
                            blocking_obstacle_id);
        }

        // remove empty path_data.
        if (!curr_path_data.Empty())
        {
            if (valid_path_cnt != i)
            {
                valid_path_data[valid_path_cnt] = curr_path_data;
            }
            ++valid_path_cnt;
        }

        // RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
        //                 reference_line_info);

#if debug_path_assessment_decider
        AINFO << "For " << curr_path_data.path_label() << ", "
              << "path length = " << curr_path_data.frenet_frame_path().size();
#endif
    }

    valid_path_data.resize(valid_path_cnt);

#if debug_path_assessment_decider
    AINFO << "valid path size: " << valid_path_data.size();
    for (size_t i = 0; i < valid_path_data.size(); i++)
    {
        AINFO << "valid path: " << valid_path_data[i].path_label()
              << " , path s: "
              << valid_path_data[i].frenet_frame_path().Length()
              << ", block obs: " << valid_path_data[i].blocking_obstacle_id();
    }


#endif

    // If there is no valid path_data, exit.
    if (valid_path_data.empty())
    {
        const std::string msg = "Neither regular nor fallback path is valid.";
        AERROR << msg;

        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    const auto& end_time2 = std::chrono::system_clock::now();
    diff = end_time2 - end_time1;

#if debug_path_assessment_decider_time
    AINFO << "Time for path info labeling: " << diff.count() * 1000
           << " msec.";
#endif

    // 3. Pick the optimal path.
    std::sort(valid_path_data.begin(), valid_path_data.end(),
              std::bind(ComparePathData, std::placeholders::_1,
                        std::placeholders::_2, blocking_obstacle_on_selflane));

    AINFO << "selected path: " << valid_path_data.front().path_label()
          << " from: " << valid_path_data.size() << " path."
          << " block obs id " << valid_path_data.front().blocking_obstacle_id();

#if debug_path_assessment_decider
    AINFO << "sl path s: "
          << valid_path_data.front().frenet_frame_path().Length();
    AINFO << "cartesian path s: "
          << valid_path_data.front().discretized_path().Length();
    AINFO << "cartesian path end point s: "
          << valid_path_data.front().discretized_path().max_s();
#endif

    if (valid_path_data.front().path_label().find("fallback") !=
        std::string::npos)
    {
        FLAGS_static_obstacle_nudge_l_buffer = 0.8;
    }

    *(reference_line_info->mutable_path_data()) = valid_path_data.front();

    // lane borrow state
    planning::LaneBorrowStatus* mutable_lane_borrow_decider =
            injector_->planning_context()
                    ->mutable_planning_status()
                    ->mutable_lane_borrow_decider();

    // manual lane borrow
    const LaneBorrowManual& lane_borrow_manual =
            mutable_lane_borrow_decider->lane_borrow_by_human();

    update_path_by_manual_lane_borrow(lane_borrow_manual, reference_line_info);

    // turtle lane borrow
    if (mutable_lane_borrow_decider->has_lane_borrow_by_turtle())
    {
        const LaneBorrowByTurtleVehicle& lane_borrow_turtle_car =
                mutable_lane_borrow_decider->lane_borrow_by_turtle();

        update_path_by_turtle_car_decision_lane_borrow(lane_borrow_turtle_car,
                                                       reference_line_info);
    }

    // blocking obs
    reference_line_info->SetBlockingObstacle(
            valid_path_data.front().blocking_obstacle_id());

    const auto& end_time3 = std::chrono::system_clock::now();
    diff = end_time3 - end_time2;

#if debug_path_assessment_decider_time
    ADEBUG << "Time for optimal path selection: " << diff.count() * 1000
           << " msec.";
#endif

    reference_line_info->SetCandidatePathData(std::move(valid_path_data));

    // 4. Update necessary info for lane-borrow decider's future uses.
    // Update front static obstacle's info.

    int front_static_obstacle_cycle_counter =
            mutable_lane_borrow_decider->front_static_obstacle_cycle_counter();

    if (reference_line_info->GetBlockingObstacle() != nullptr)
    {

        mutable_lane_borrow_decider->set_front_static_obstacle_cycle_counter(
                std::max(front_static_obstacle_cycle_counter, 0));

        mutable_lane_borrow_decider->set_front_static_obstacle_cycle_counter(
                std::min(front_static_obstacle_cycle_counter + 1, 10));

        mutable_lane_borrow_decider->set_front_static_obstacle_id(
                reference_line_info->GetBlockingObstacle()->Id());

        AINFO << "mutable_path_decider obs id "
              << mutable_lane_borrow_decider->front_static_obstacle_id();
    }
    else
    {
        mutable_lane_borrow_decider->set_front_static_obstacle_cycle_counter(
                std::min(front_static_obstacle_cycle_counter, 0));

        mutable_lane_borrow_decider->set_front_static_obstacle_cycle_counter(
                std::max(front_static_obstacle_cycle_counter - 1, -10));
    }

    // Update self-lane usage info.
    if (reference_line_info->path_data().path_label().find("self") !=
        std::string::npos)
    {
        // && std::get<1>(reference_line_info->path_data()
        //                 .path_point_decision_guide()
        //                 .front()) == PathData::PathPointType::IN_LANE)

        int able_to_use_self_lane_counter =
                mutable_lane_borrow_decider->able_to_use_self_lane_counter();

        if (able_to_use_self_lane_counter < 0)
        {
            able_to_use_self_lane_counter = 0;
        }

        // 没有阻挡obs，那么lane keep path counter +1
        if (blocking_obstacle_on_selflane == nullptr ||
            blocking_obstacle_on_selflane->IsVirtual())
        {
            able_to_use_self_lane_counter =
                    std::min(able_to_use_self_lane_counter + 1, 10);

            // AINFO << "lane keep path can use, +1: "
            //       << mutable_lane_borrow_decider->front_static_obstacle_id();
        }
        else
        {
            able_to_use_self_lane_counter = 0;
        }

        mutable_lane_borrow_decider->set_able_to_use_self_lane_counter(
                able_to_use_self_lane_counter);
    }
    else
    {
        mutable_lane_borrow_decider->set_able_to_use_self_lane_counter(0);
    }

    // Update side-pass direction.
    if (mutable_lane_borrow_decider->is_in_path_lane_borrow_scenario())
    {
        bool left_borrow = false;
        bool right_borrow = false;

        // const auto& path_decider_status =
        // injector_->planning_context()
        //         ->planning_status()
        //         .lane_borrow_decider();

        // for (const auto& lane_borrow_direction :
        //      path_decider_status.decided_side_pass_direction())
        // {
            // if (lane_borrow_direction == LaneBorrowStatus::LEFT_BORROW &&
            //     reference_line_info->path_data().path_label().find("left") !=
            //             std::string::npos)
            // {
            //     left_borrow = true;
            // }


            // if (lane_borrow_direction == LaneBorrowStatus::RIGHT_BORROW &&
            //     reference_line_info->path_data().path_label().find("right") !=
            //             std::string::npos)
            // {
            //     right_borrow = true;
            // }

        // }

        if (reference_line_info->has_lane_borrow_path(
                    LANE_BORROW_DIRECTION_LEFT))
        {
            left_borrow = true;
        }

        if (reference_line_info->has_lane_borrow_path(
                    LANE_BORROW_DIRECTION_RIGHT))
        {
            right_borrow = true;
        }

        // 如果生成的lane borrow path都无效，那么clear lane borrow
        // direction，导致再也不能借道，这里有bug,todo
        mutable_lane_borrow_decider->clear_decided_side_pass_direction();
        if (right_borrow)
        {
            mutable_lane_borrow_decider->add_decided_side_pass_direction(
                    LANE_BORROW_DIRECTION_RIGHT);
        }
        if (left_borrow)
        {
            mutable_lane_borrow_decider->add_decided_side_pass_direction(
                    LANE_BORROW_DIRECTION_LEFT);
        }
    }
    const auto& end_time4 = std::chrono::system_clock::now();
    diff = end_time4 - end_time3;

#if debug_path_assessment_decider_time
    AINFO << "Time for FSM state updating: " << diff.count() * 1000
           << " msec.";
#endif

    // Plot the path in simulator for debug purpose.
    RecordDebugInfo(reference_line_info->path_data(), "PlanningPathData",
                    reference_line_info);

    return Status::OK();
}

// left side hand
// 左侧更好，返回true
bool ComparePathData(const PathData& lhs, const PathData& rhs,
                     const Obstacle* blocking_obstacle)
{
    ADEBUG << "Comparing " << lhs.path_label() << " and " << rhs.path_label();

    // Empty path_data is never the larger one.
    if (lhs.Empty())
    {
        ADEBUG << "LHS is empty.";
        return false;
    }
    if (rhs.Empty())
    {
        ADEBUG << "RHS is empty.";
        return true;
    }

    // Regular path goes before fallback path. 一般情况下，不选择fallback path
    bool lhs_is_regular = lhs.path_label().find("regular") != std::string::npos;
    bool rhs_is_regular = rhs.path_label().find("regular") != std::string::npos;
    if (lhs_is_regular != rhs_is_regular)
    {
        return lhs_is_regular;
    }

    // Select longer path.
    // If roughly same length, then select self-lane path.
    bool lhs_on_selflane = lhs.path_label().find("self") != std::string::npos;
    bool rhs_on_selflane = rhs.path_label().find("self") != std::string::npos;
    static constexpr double kSelfPathLengthComparisonTolerance = 15.0;
    static constexpr double kNeighborPathLengthComparisonTolerance = 25.0;

    double lhs_path_length = lhs.frenet_frame_path().back().s();
    double rhs_path_length = rhs.frenet_frame_path().back().s();
    
    if (lhs_on_selflane || rhs_on_selflane)
    {
        // 选择更长的
        if (std::fabs(lhs_path_length - rhs_path_length) >
            kSelfPathLengthComparisonTolerance)
        {
            return lhs_path_length > rhs_path_length;
        }
        // 长度相似，选择自车道轨迹
        else
        {
            return lhs_on_selflane;
        }
    }
    // both lane borrow
    else
    {
        // 路径更长，那么reward更高
        if (std::fabs(lhs_path_length - rhs_path_length) >
            kNeighborPathLengthComparisonTolerance)
        {
            return lhs_path_length > rhs_path_length;
        }
    }

    // If roughly same length, and must borrow neighbor lane,
    // then prefer to borrow forward lane rather than reverse lane.
    int lhs_on_reverse =
            ContainsOutOnReverseLane(lhs.path_point_decision_guide());
    int rhs_on_reverse =
            ContainsOutOnReverseLane(rhs.path_point_decision_guide());

    // TODO(jiacheng): make this a flag.
    if (std::abs(lhs_on_reverse - rhs_on_reverse) > 6)
    {
        return lhs_on_reverse < rhs_on_reverse;
    }

    // For two lane-borrow directions, based on ADC's position,
    // select the more convenient one.
    if ((lhs.path_label().find("left") != std::string::npos &&
         rhs.path_label().find("right") != std::string::npos) ||
        (lhs.path_label().find("right") != std::string::npos &&
         rhs.path_label().find("left") != std::string::npos))
    {
        if (blocking_obstacle)
        {
            // select left/right path based on blocking_obstacle's position
            const double obstacle_l =
                    (blocking_obstacle->PerceptionSLBoundary().start_l() +
                     blocking_obstacle->PerceptionSLBoundary().end_l()) /
                    2;
            ADEBUG << "obstacle[" << blocking_obstacle->Id() << "] l["
                   << obstacle_l << "]";
            return (obstacle_l > 0.0 ? (lhs.path_label().find("right") !=
                                        std::string::npos)
                                     : (lhs.path_label().find("left") !=
                                        std::string::npos));
        }
        else
        {
            // select left/right path based on ADC's position
            double adc_l = lhs.frenet_frame_path().front().l();
            if (adc_l < -1.0)
            {
                return lhs.path_label().find("right") != std::string::npos;
            }
            else if (adc_l > 1.0)
            {
                return lhs.path_label().find("left") != std::string::npos;
            }
        }
    }

    // If same length, both neighbor lane are forward,
    // then select the one that returns to in-lane earlier.
    static constexpr double kBackToSelfLaneComparisonTolerance = 20.0;
    int lhs_back_idx = GetBackToInLaneIndex(lhs.path_point_decision_guide());
    int rhs_back_idx = GetBackToInLaneIndex(rhs.path_point_decision_guide());
    double lhs_back_s = lhs.frenet_frame_path()[lhs_back_idx].s();
    double rhs_back_s = rhs.frenet_frame_path()[rhs_back_idx].s();
    if (std::fabs(lhs_back_s - rhs_back_s) > kBackToSelfLaneComparisonTolerance)
    {
        return lhs_back_idx < rhs_back_idx;
    }

    // If same length, both forward, back to inlane at same time,
    // select the left one to side-pass.
    bool lhs_on_leftlane = lhs.path_label().find("left") != std::string::npos;
    bool rhs_on_leftlane = rhs.path_label().find("left") != std::string::npos;

    if (lhs_on_leftlane != rhs_on_leftlane)
    {
        ADEBUG << "Select " << (lhs_on_leftlane ? "left" : "right")
               << " lane over " << (!lhs_on_leftlane ? "left" : "right")
               << " lane.";
        return lhs_on_leftlane;
    }

    // Otherwise, they are the same path, lhs is not < rhs.
    return false;
}

bool PathAssessmentDecider::IsValidRegularPath(
        const ReferenceLineInfo& reference_line_info, const PathData& path_data)
{
    // Basic sanity checks.
    if (path_data.Empty())
    {
        ADEBUG << path_data.path_label() << ": path data is empty.";
        return false;
    }
    // Check if the path is greatly off the reference line.
    if (IsGreatlyOffReferenceLine(path_data))
    {
        ADEBUG << path_data.path_label()
               << ": ADC is greatly off reference line.";
        return false;
    }
    // Check if the path is greatly off the road.
    if (IsGreatlyOffRoad(reference_line_info, path_data))
    {
        ADEBUG << path_data.path_label() << ": ADC is greatly off road.";
        return false;
    }

#if debug_path_assessment_decider_time
    const auto& end_time0 = std::chrono::system_clock::now();
#endif

    // Check if there is any collision.
    // if (IsCollidingWithStaticObstacles(reference_line_info, path_data))
    if (is_colliding_with_static_obstacles(reference_line_info, path_data))
    {
#if debug_path_assessment_decider
        AINFO << path_data.path_label() << ": ADC has collision.";
#endif

        return false;
    }

#if debug_path_assessment_decider_time
    const auto& end_time1 = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_time1 - end_time0;

    AINFO << "Time for path collision checking: " << diff.count() * 1000
          << " msec.";
#endif

    if (IsStopOnReverseNeighborLane(reference_line_info, path_data))
    {
        ADEBUG << path_data.path_label() << ": stop at reverse neighbor lane";

        return false;
    }

    return true;
}

bool PathAssessmentDecider::IsValidFallbackPath(
        const ReferenceLineInfo& reference_line_info, const PathData& path_data)
{
    // Basic sanity checks.
    if (path_data.Empty())
    {
        ADEBUG << "Fallback Path: path data is empty.";
        return false;
    }

    // Check if the path is greatly off the reference line.
    if (IsGreatlyOffReferenceLine(path_data))
    {
        ADEBUG << "Fallback Path: ADC is greatly off reference line.";
        return false;
    }

    // Check if the path is greatly off the road.
    if (IsGreatlyOffRoad(reference_line_info, path_data))
    {
        ADEBUG << "Fallback Path: ADC is greatly off road.";

        return false;
    }

    return true;
}

void PathAssessmentDecider::SetPathInfo(
        const ReferenceLineInfo& reference_line_info, PathData* const path_data)
{
    // Go through every path_point, and label its:
    //  - in-lane/out-of-lane info (side-pass or lane-change)
    //  - distance to the closest obstacle.
    std::vector<PathPointDecision> path_decision;

    // 0. Initialize the path info.
    // 初始化每一个path point的决策
    InitPathPointDecision(*path_data, &path_decision);

    // 1. Label caution types, differently for side-pass or lane-change.
    if (reference_line_info.is_change_lane_ref_line())
    {
        // If lane-change, then label the lane-changing part to
        // be out-on-forward lane.

        // SetPathPointType(reference_line_info, *path_data, true, &path_decision);
        check_path_cross_lane_line(reference_line_info, *path_data, true,
                                   &path_decision);
    }
    else
    {
        // Otherwise, only do the label for borrow-lane generated paths.
        // 只对lane borrow path进行计算
        if (path_data->path_label().find("fallback") == std::string::npos &&
            path_data->path_label().find("self") == std::string::npos)
        {
            // SetPathPointType(reference_line_info, *path_data, false,
            //                  &path_decision);
            check_path_cross_lane_line(reference_line_info, *path_data, false,
                                       &path_decision);
        }
    }

    // SetObstacleDistance(reference_line_info, *path_data, &path_decision);
    path_data->SetPathPointDecisionGuide(std::move(path_decision));
}

void PathAssessmentDecider::TrimTailingOutLanePoints(PathData* const path_data)
{
    // Don't trim self-lane path or fallback path.
    if (path_data->path_label().find("fallback") != std::string::npos ||
        path_data->path_label().find("self") != std::string::npos)
    {
        return;
    }

#if debug_path_assessment_decider_time
    const auto& trim_start = std::chrono::system_clock::now();
#endif

    // Trim.
#if debug_path_assessment_decider
    AINFO << "Trimming " << path_data->path_label();
#endif

    auto frenet_path = path_data->frenet_frame_path();
    auto path_point_decision = path_data->path_point_decision_guide();

    CHECK_EQ(frenet_path.size(), path_point_decision.size());

    while (!path_point_decision.empty() &&
           std::get<1>(path_point_decision.back()) !=
                   PathData::PathPointType::IN_LANE)
    {
#if debug_path_assessment_decider
        if (std::get<1>(path_point_decision.back()) ==
            PathData::PathPointType::OUT_ON_FORWARD_LANE)
        {
            AINFO << "Trimming out forward lane point";
        }
        else if (std::get<1>(path_point_decision.back()) ==
                 PathData::PathPointType::OUT_ON_REVERSE_LANE)
        {
            AINFO << "Trimming out reverse lane point";
        }
        else
        {
            AINFO << "Trimming unknown lane point";
        }
#endif
        frenet_path.pop_back();
        path_point_decision.pop_back();
    }

    path_data->SetFrenetPath(std::move(frenet_path));
    path_data->SetPathPointDecisionGuide(std::move(path_point_decision));

#if debug_path_assessment_decider
    AINFO << "Trimming path end size: " << frenet_path.size();
#endif

#if debug_path_assessment_decider_time
    const auto& trim_end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = trim_end - trim_start;

    AINFO << "Time for path trim checking: " << diff.count() * 1000 << " msec.";
#endif

    // discretized_path_ 这个怎么不修剪？
}

bool PathAssessmentDecider::IsGreatlyOffReferenceLine(const PathData& path_data)
{
    static constexpr double kOffReferenceLineThreshold = 20.0;
    const auto& frenet_path = path_data.frenet_frame_path();
    for (const auto& frenet_path_point : frenet_path)
    {
        if (std::fabs(frenet_path_point.l()) > kOffReferenceLineThreshold)
        {
            ADEBUG << "Greatly off reference line at s = "
                   << frenet_path_point.s()
                   << ", with l = " << frenet_path_point.l();

            return true;
        }
    }
    return false;
}

bool PathAssessmentDecider::IsGreatlyOffRoad(
        const ReferenceLineInfo& reference_line_info, const PathData& path_data)
{
    static constexpr double kOffRoadThreshold = 10.0;
    const auto& frenet_path = path_data.frenet_frame_path();
    for (const auto& frenet_path_point : frenet_path)
    {
        double road_left_width = 0.0;
        double road_right_width = 0.0;
        if (reference_line_info.reference_line().GetRoadWidth(
                    frenet_path_point.s(), &road_left_width, &road_right_width))
        {
            if (frenet_path_point.l() > road_left_width + kOffRoadThreshold ||
                frenet_path_point.l() < -road_right_width - kOffRoadThreshold)
            {
                ADEBUG << "Greatly off-road at s = " << frenet_path_point.s()
                       << ", with l = " << frenet_path_point.l();

                return true;
            }
        }
    }
    return false;
}

bool PathAssessmentDecider::IsCollidingWithStaticObstacles(
        const ReferenceLineInfo& reference_line_info, const PathData& path_data)
{
    // Get all obstacles and convert them into frenet-frame polygons.
    std::vector<Polygon2d> obstacle_polygons;
    const auto& indexed_obstacles =
            reference_line_info.path_decision().obstacles();

    for (const auto* obstacle : indexed_obstacles.Items())
    {
        // Filter out unrelated obstacles.
        if (!IsWithinPathDeciderScopeObstacle(*obstacle))
        {
            continue;
        }

        // Ignore too small obstacles.
        const auto& obstacle_sl = obstacle->PerceptionSLBoundary();

        if ((obstacle_sl.end_s() - obstacle_sl.start_s()) *
                    (obstacle_sl.end_l() - obstacle_sl.start_l()) <
            kMinObstacleArea)
        {
            continue;
        }

        // Convert into polygon and save it.
        obstacle_polygons.push_back(
                Polygon2d({Vec2d(obstacle_sl.start_s(), obstacle_sl.start_l()),
                           Vec2d(obstacle_sl.start_s(), obstacle_sl.end_l()),
                           Vec2d(obstacle_sl.end_s(), obstacle_sl.end_l()),
                           Vec2d(obstacle_sl.end_s(), obstacle_sl.start_l())}));
    }

    if (obstacle_polygons.size() < 1)
    {
        return false;
    }

    double path_end_s = path_data.frenet_frame_path().back().s();

    double delta_s ;

    // 当path 被障碍物阻挡时，已经会拓展一些path，此时path 末端collision with
    // obstacle.
    double collision_ignore_dist =
            (kNumExtraTailBoundPoint + 1) * kPathBoundsDeciderResolution;

    // Go through all the four corner points at every path pt, check collision.
    for (size_t i = 0; i < path_data.discretized_path().size(); ++i)
    {
        // 最后一段距离内，不做碰撞检查
        delta_s = path_end_s - path_data.frenet_frame_path()[i].s();

        if (delta_s < collision_ignore_dist)
        {
            break;
        }

        const auto& path_point = path_data.discretized_path()[i];

        // Get the four corner points ABCD of ADC at every path point.
        const auto& vehicle_box =
                common::VehicleConfigHelper::Instance()->GetBoundingBox(
                        path_point);
        std::vector<Vec2d> ABCDpoints = vehicle_box.GetAllCorners();

        for (const auto& corner_point : ABCDpoints)
        {
            // For each corner point, project it onto reference_line
            common::SLPoint curr_point_sl;
            if (!reference_line_info.reference_line().XYToSL(corner_point,
                                                             &curr_point_sl))
            {
                AERROR << "Failed to get the projection from point onto "
                          "reference_line";
                return true;
            }

            auto curr_point = Vec2d(curr_point_sl.s(), curr_point_sl.l());

            // Check if it's in any polygon of other static obstacles.
            for (const auto& obstacle_polygon : obstacle_polygons)
            {
                if (obstacle_polygon.IsPointIn(curr_point))
                {
                    ADEBUG << "ADC is colliding with obstacle at path s = "
                           << path_point.s();
                    return true;
                }
            }
        }
    }

    return false;
}

bool PathAssessmentDecider::is_colliding_with_static_obstacles(
        const ReferenceLineInfo& reference_line_info, const PathData& path_data)
{
    // Get all obstacles and convert them into frenet-frame polygons.
    std::vector<Polygon2D> obstacle_polygons;
    const auto& indexed_obstacles =
            reference_line_info.path_decision().obstacles();

    Polygon2D obs_polygon;
    for (const auto* obstacle : indexed_obstacles.Items())
    {
        // Filter out unrelated obstacles.
        if (!IsWithinPathDeciderScopeObstacle(*obstacle))
        {
            continue;
        }

        // Ignore too small obstacles.
        const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
        if ((obstacle_sl.end_s() - obstacle_sl.start_s()) *
                    (obstacle_sl.end_l() - obstacle_sl.start_l()) <
            kMinObstacleArea)
        {
            continue;
        }

        const common::math::Box2d& obs_box = obstacle->PerceptionBoundingBox();


        cvt_box2d_to_polygon(&obs_polygon, obs_box);

        // Convert into polygon and save it.
        obstacle_polygons.emplace_back(obs_polygon);
    }

    if (obstacle_polygons.size() < 1)
    {
        return false;
    }

    double path_end_s = path_data.frenet_frame_path().back().s();

    double delta_s;

    // 当path 被障碍物阻挡时，已经会拓展一些path，此时path 末端collision with
    // obstacle. 会忽略这段距离内的检查。
    // todo： 检查整条path, 选择path 最长的那一条。
    double collision_ignore_dist =
            (kNumExtraTailBoundPoint + 1) * kPathBoundsDeciderResolution;

    int path_check_size = 0;
    int path_max_size = path_data.discretized_path().size();

    // Go through all the four corner points at every path pt, check collision.
    for (size_t i = 0; i < path_max_size; ++i)
    {
        // 最后一段距离内，不做碰撞检查
        delta_s = path_end_s - path_data.frenet_frame_path()[i].s();

        if (delta_s < collision_ignore_dist)
        {
            break;
        }
        path_check_size = i;
    }

    // generate path
    std::vector<Polygon2D> adc_polygon_path_;
    generate_polygon_path(path_assessment_safe_buffer,
                          path_data.discretized_path(), adc_polygon_path_,
                          path_check_size);

    if (adc_polygon_path_.size() < 1)
    {
        AERROR << "path polygon size is 0";
        return false;
    }

    bool is_collision = false;

    // Check if it's in any polygon of other static obstacles.
    for (const auto& obstacle_polygon : obstacle_polygons)
    {
        is_collision = is_path_collision_with_static_obstacle(adc_polygon_path_,
                                                              obstacle_polygon);

        if (is_collision)
        {
            break;
        }
    }

    return is_collision;
}

bool PathAssessmentDecider::IsStopOnReverseNeighborLane(
        const ReferenceLineInfo& reference_line_info, const PathData& path_data)
{
    if (path_data.path_label().find("left") == std::string::npos &&
        path_data.path_label().find("right") == std::string::npos)
    {
        return false;
    }

    std::vector<common::SLPoint> all_stop_point_sl =
            reference_line_info.GetAllStopDecisionSLPoint();
    if (all_stop_point_sl.empty())
    {
        return false;
    }

    double check_s = 0.0;
    // filter out sidepass stop fence
    static constexpr double kLookForwardBuffer = 5.0;
    const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
    for (const auto& stop_point_sl : all_stop_point_sl)
    {
        if (stop_point_sl.s() - adc_end_s < kLookForwardBuffer)
        {
            continue;
        }
        check_s = stop_point_sl.s();
        break;
    }
    if (check_s <= 0.0)
    {
        return false;
    }

    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    if (!reference_line_info.reference_line().GetLaneWidth(
                check_s, &lane_left_width, &lane_right_width))
    {
        return false;
    }

    // 首先得到停车点，然后找到停车点附近的路径点
    static constexpr double kSDelta = 0.3;
    common::SLPoint path_point_sl;
    for (const auto& frenet_path_point : path_data.frenet_frame_path())
    {
        if (std::fabs(frenet_path_point.s() - check_s) < kSDelta)
        {
            path_point_sl.set_s(frenet_path_point.s());
            path_point_sl.set_l(frenet_path_point.l());
        }
    }
    ADEBUG << "path_point_sl[" << path_point_sl.s() << ", " << path_point_sl.l()
           << "] lane_left_width[" << lane_left_width << "] lane_right_width["
           << lane_right_width << "]";

    hdmap::Id neighbor_lane_id;
    double neighbor_lane_width = 0.0;
    if (path_data.path_label().find("left") != std::string::npos &&
        path_point_sl.l() > lane_left_width)
    {
        if (reference_line_info.GetNeighborLaneInfo(
                    ReferenceLineInfo::LaneType::LeftReverse, path_point_sl.s(),
                    &neighbor_lane_id, &neighbor_lane_width))
        {
            ADEBUG << "stop path point at LeftReverse neighbor lane["
                   << neighbor_lane_id.id() << "]";
            return true;
        }
    }
    else if (path_data.path_label().find("right") != std::string::npos &&
             path_point_sl.l() < -lane_right_width)
    {
        if (reference_line_info.GetNeighborLaneInfo(
                    ReferenceLineInfo::LaneType::RightReverse,
                    path_point_sl.s(), &neighbor_lane_id, &neighbor_lane_width))
        {
            ADEBUG << "stop path point at RightReverse neighbor lane["
                   << neighbor_lane_id.id() << "]";
            return true;
        }
    }
    return false;
}

void PathAssessmentDecider::InitPathPointDecision(
        const PathData& path_data,
        std::vector<PathPointDecision>* const path_point_decision)
{
    // Sanity checks.
    CHECK_NOTNULL(path_point_decision);
    path_point_decision->clear();

    // Go through every path point in path data, and initialize a
    // corresponding path point decision.
    for (const auto& frenet_path_point : path_data.frenet_frame_path())
    {
        path_point_decision->emplace_back(frenet_path_point.s(),
                                          PathData::PathPointType::UNKNOWN,
                                          std::numeric_limits<double>::max());
    }
}

void PathAssessmentDecider::SetPathPointType(
        const ReferenceLineInfo& reference_line_info, const PathData& path_data,
        const bool is_lane_change_path,
        std::vector<PathPointDecision>* const path_point_decision)
{
    // Sanity checks.
    CHECK_NOTNULL(path_point_decision);

    // Go through every path_point, and add in-lane/out-of-lane info.
    const auto& discrete_path = path_data.discretized_path();
    const auto& vehicle_config =
            common::VehicleConfigHelper::Instance()->GetConfig();
    const double ego_length = vehicle_config.vehicle_param().length();
    const double ego_width = vehicle_config.vehicle_param().width();

    const double ego_back_to_center =
            vehicle_config.vehicle_param().back_edge_to_center();
    const double ego_center_shift_distance =
            ego_length / 2.0 - ego_back_to_center;

    bool is_prev_point_out_lane = false;
    for (size_t i = 0; i < discrete_path.size(); ++i)
    {
        const auto& rear_center_path_point = discrete_path[i];
        const double ego_theta = rear_center_path_point.theta();
        Box2d ego_box({rear_center_path_point.x(), rear_center_path_point.y()},
                      ego_theta, ego_length, ego_width);

        Vec2d shift_vec{ego_center_shift_distance * std::cos(ego_theta),
                        ego_center_shift_distance * std::sin(ego_theta)};

        ego_box.Shift(shift_vec);

        SLBoundary ego_sl_boundary;
        if (!reference_line_info.reference_line().GetSLBoundary(
                    ego_box, &ego_sl_boundary))
        {
            ADEBUG << "Unable to get SL-boundary of ego-vehicle.";
            continue;
        }

        double lane_left_width = 0.0;
        double lane_right_width = 0.0;
        double middle_s =
                (ego_sl_boundary.start_s() + ego_sl_boundary.end_s()) / 2.0;

        if (reference_line_info.reference_line().GetLaneWidth(
                    middle_s, &lane_left_width, &lane_right_width))
        {
            // Rough sl boundary estimate using single point lane width
            double back_to_inlane_extra_buffer = 0.2;
            double in_and_out_lane_hysteresis_buffer =
                    is_prev_point_out_lane ? back_to_inlane_extra_buffer : 0.0;

            // Check for lane-change and lane-borrow differently:
            if (is_lane_change_path)
            {
                // For lane-change path, only transitioning part is labeled as
                // out-of-lane.
                if (ego_sl_boundary.start_l() > lane_left_width ||
                    ego_sl_boundary.end_l() < -lane_right_width)
                {
                    // This means that ADC hasn't started lane-change yet.
                    std::get<1>((*path_point_decision)[i]) =
                            PathData::PathPointType::IN_LANE;
                }
                else if (ego_sl_boundary.start_l() >
                                 -lane_right_width +
                                         back_to_inlane_extra_buffer &&
                         ego_sl_boundary.end_l() <
                                 lane_left_width - back_to_inlane_extra_buffer)
                {
                    // This means that ADC has safely completed lane-change with
                    // margin.
                    std::get<1>((*path_point_decision)[i]) =
                            PathData::PathPointType::IN_LANE;
                }
                else
                {
                    // ADC is right across two lanes.
                    std::get<1>((*path_point_decision)[i]) =
                            PathData::PathPointType::OUT_ON_FORWARD_LANE;
                }
            }
            else
            {
                // For lane-borrow path, as long as ADC is not on the lane of
                // reference-line, it is out on other lanes. It might even be
                // on reverse lane!
                if (ego_sl_boundary.end_l() >
                            lane_left_width +
                                    in_and_out_lane_hysteresis_buffer ||
                    ego_sl_boundary.start_l() <
                            -lane_right_width -
                                    in_and_out_lane_hysteresis_buffer)
                {
                    if (path_data.path_label().find("reverse") !=
                        std::string::npos)
                    {
                        std::get<1>((*path_point_decision)[i]) =
                                PathData::PathPointType::OUT_ON_REVERSE_LANE;
                    }
                    else if (path_data.path_label().find("forward") !=
                             std::string::npos)
                    {
                        std::get<1>((*path_point_decision)[i]) =
                                PathData::PathPointType::OUT_ON_FORWARD_LANE;
                    }
                    else
                    {
                        std::get<1>((*path_point_decision)[i]) =
                                PathData::PathPointType::UNKNOWN;
                    }

                    if (!is_prev_point_out_lane)
                    {
                        if (ego_sl_boundary.end_l() >
                                    lane_left_width +
                                            back_to_inlane_extra_buffer ||
                            ego_sl_boundary.start_l() <
                                    -lane_right_width -
                                            back_to_inlane_extra_buffer)
                        {
                            is_prev_point_out_lane = true;
                        }
                    }
                }
                else
                {
                    // The path point is within the reference_line's lane.
                    std::get<1>((*path_point_decision)[i]) =
                            PathData::PathPointType::IN_LANE;
                    if (is_prev_point_out_lane)
                    {
                        is_prev_point_out_lane = false;
                    }
                }
            }
        }
        else
        {
            AERROR << "reference line not ready when setting path point guide";
            return;
        }
    }

    return;
}

void PathAssessmentDecider::check_path_cross_lane_line(
        const ReferenceLineInfo& reference_line_info, const PathData& path_data,
        const bool is_lane_change_path,
        std::vector<PathPointDecision>* const path_point_state)
{
    // Sanity checks.
    if (path_point_state == nullptr)
    {
        return;
    }

    // Go through every path_point, and add in-lane/out-of-lane info.
    const auto& discrete_path = path_data.discretized_path();
    const FrenetFramePath& sl_path = path_data.frenet_frame_path();

    const ReferenceLine& ref_line = reference_line_info.reference_line();

    const auto& vehicle_config =
            common::VehicleConfigHelper::Instance()->GetConfig();

    const double ego_length = vehicle_config.vehicle_param().length();
    const double ego_width = vehicle_config.vehicle_param().width();
    const double ego_half_width = ego_width * 0.5;

    const double ego_back_to_center =
            vehicle_config.vehicle_param().back_edge_to_center();

    const double ego_front_to_center =
            vehicle_config.vehicle_param().front_edge_to_center();

    const double ego_center_shift_distance =
            ego_length / 2.0 - ego_back_to_center;


    bool is_prev_point_out_lane = false;
    int path_point_size = sl_path.size();

    double ego_theta;

    ReferencePoint ref_point;

    double point_delta_theta;

    SLBoundary point_sl_boundary;

    double point_s;
    double point_l;

    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    double middle_s;

    // Rough sl boundary estimate using single point lane width
    double back_to_inlane_extra_buffer = 0.2;

    for (size_t i = 0; i < path_point_size; ++i)
    {
        const common::PathPoint& rear_center_path_point = discrete_path[i];
        const common::FrenetFramePoint& sl_path_point = sl_path[i];

        // init
        point_s = sl_path_point.s();

        ego_theta = rear_center_path_point.theta();

        ref_point = ref_line.GetReferencePoint(sl_path_point.s());

        point_delta_theta = ego_theta - ref_point.heading();

        // generate sl bound
        get_sl_boundary_by_delta_theta(&point_sl_boundary, ego_half_width,
                                       ego_front_to_center, ego_back_to_center,
                                       point_delta_theta, sl_path_point);

        middle_s =
                (point_sl_boundary.start_s() + point_sl_boundary.end_s()) / 2.0;

        // check path across lane line
        if (!ref_line.GetLaneWidth(middle_s, &lane_left_width,
                                  &lane_right_width))
        {
            AERROR << "reference line not ready when setting path point guide"
                   << ", middle_s: " << middle_s;

            return;
        }


        double in_and_out_lane_hysteresis_buffer =
                is_prev_point_out_lane ? back_to_inlane_extra_buffer : 0.0;

        // Check for lane-change and lane-borrow differently:
        if (is_lane_change_path)
        {
            // For lane-change path, only transitioning part is labeled as
            // out-of-lane.
            if (point_sl_boundary.start_l() > lane_left_width ||
                point_sl_boundary.end_l() < -lane_right_width)
            {
                // This means that ADC hasn't started lane-change yet.
                std::get<1>((*path_point_state)[i]) =
                        PathData::PathPointType::IN_LANE;
            }
            else if (point_sl_boundary.start_l() >
                             -lane_right_width + back_to_inlane_extra_buffer &&
                     point_sl_boundary.end_l() <
                             lane_left_width - back_to_inlane_extra_buffer)
            {
                // This means that ADC has safely completed lane-change with
                // margin.
                std::get<1>((*path_point_state)[i]) =
                        PathData::PathPointType::IN_LANE;
            }
            else
            {
                // ADC is right across two lanes.
                std::get<1>((*path_point_state)[i]) =
                        PathData::PathPointType::OUT_ON_FORWARD_LANE;
            }
        }
        else
        {
            // For lane-borrow path, as long as ADC is not on the lane of
            // reference-line, it is out on other lanes. It might even be
            // on reverse lane!
            if (point_sl_boundary.end_l() >
                        lane_left_width + in_and_out_lane_hysteresis_buffer ||
                point_sl_boundary.start_l() <
                        -lane_right_width - in_and_out_lane_hysteresis_buffer)
            {
                if (path_data.path_label().find("reverse") != std::string::npos)
                {
                    std::get<1>((*path_point_state)[i]) =
                            PathData::PathPointType::OUT_ON_REVERSE_LANE;
                }
                else if (path_data.path_label().find("forward") !=
                         std::string::npos)
                {
                    std::get<1>((*path_point_state)[i]) =
                            PathData::PathPointType::OUT_ON_FORWARD_LANE;
                }
                else
                {
                    std::get<1>((*path_point_state)[i]) =
                            PathData::PathPointType::UNKNOWN;
                }

                if (!is_prev_point_out_lane)
                {
                    if (point_sl_boundary.end_l() >
                                lane_left_width + back_to_inlane_extra_buffer ||
                        point_sl_boundary.start_l() <
                                -lane_right_width - back_to_inlane_extra_buffer)
                    {
                        is_prev_point_out_lane = true;
                    }
                }
            }
            else
            {
                // The path point is within the reference_line's lane.
                std::get<1>((*path_point_state)[i]) =
                        PathData::PathPointType::IN_LANE;

                if (is_prev_point_out_lane)
                {
                    is_prev_point_out_lane = false;
                }
            }
        }
    }

    return;
}

void PathAssessmentDecider::SetObstacleDistance(
        const ReferenceLineInfo& reference_line_info, const PathData& path_data,
        std::vector<PathPointDecision>* const path_point_decision)
{
    // Sanity checks
    CHECK_NOTNULL(path_point_decision);

    // Get all obstacles and convert them into frenet-frame polygons.
    std::vector<Polygon2d> obstacle_polygons;
    const auto& indexed_obstacles =
            reference_line_info.path_decision().obstacles();
    for (const auto* obstacle : indexed_obstacles.Items())
    {
        // Filter out unrelated obstacles.
        if (!IsWithinPathDeciderScopeObstacle(*obstacle))
        {
            continue;
        }
        // Convert into polygon and save it.
        const auto& obstacle_box = obstacle->PerceptionBoundingBox();
        if (obstacle_box.area() < kMinObstacleArea)
        {
            continue;
        }
        obstacle_polygons.emplace_back(obstacle_box);
    }

    // Go through every path point, update closest obstacle info.
    const auto& discrete_path = path_data.discretized_path();
    for (size_t i = 0; i < discrete_path.size(); ++i)
    {
        const auto& path_point = discrete_path[i];
        // Get the bounding box of the vehicle at that point.
        const auto& vehicle_box =
                common::VehicleConfigHelper::Instance()->GetBoundingBox(
                        path_point);
        // Go through all the obstacle polygons, and update the min distance.
        double min_distance_to_obstacles = std::numeric_limits<double>::max();
        for (const auto& obstacle_polygon : obstacle_polygons)
        {
            double distance_to_vehicle =
                    obstacle_polygon.DistanceTo(vehicle_box);
            min_distance_to_obstacles =
                    std::min(min_distance_to_obstacles, distance_to_vehicle);
        }
        std::get<2>((*path_point_decision)[i]) = min_distance_to_obstacles;
    }
}

void PathAssessmentDecider::RecordDebugInfo(
        const PathData& path_data, const std::string& debug_name,
        ReferenceLineInfo* const reference_line_info)
{
    const auto& path_points = path_data.discretized_path();
    auto* ptr_optimized_path = reference_line_info->mutable_debug()
                                       ->mutable_planning_data()
                                       ->add_path();

    ptr_optimized_path->set_name(debug_name);
    ptr_optimized_path->mutable_path_point()->CopyFrom(
            {path_points.begin(), path_points.end()});

    if (path_points.size() > 1)
    {
        const auto& sl_path_points = path_data.frenet_frame_path();

        int size = sl_path_points.size();

        const common::FrenetFramePoint& sl_path_point =
                sl_path_points[size - 1];

        AINFO << "print_path_end_lateral_lane_offet:"
              << "(" << sl_path_point.l() << ","
              << ")";
    }
}

int ContainsOutOnReverseLane(
        const std::vector<PathPointDecision>& path_point_decision)
{
    int ret = 0;
    for (const auto& curr_decision : path_point_decision)
    {
        if (std::get<1>(curr_decision) ==
            PathData::PathPointType::OUT_ON_REVERSE_LANE)
        {
            ++ret;
        }
    }
    return ret;
}

int GetBackToInLaneIndex(
        const std::vector<PathPointDecision>& path_point_decision)
{
    // ACHECK(!path_point_decision.empty());
    // ACHECK(std::get<1>(path_point_decision.back()) ==
    //       PathData::PathPointType::IN_LANE);

    for (int i = static_cast<int>(path_point_decision.size()) - 1; i >= 0; --i)
    {
        if (std::get<1>(path_point_decision[i]) !=
            PathData::PathPointType::IN_LANE)
        {
            return i;
        }
    }
    return 0;
}

int PathAssessmentDecider::update_path_by_manual_lane_borrow(
        const LaneBorrowManual& lane_borrow_manual,
        ReferenceLineInfo* const reference_line_info)
{
    if (!lane_borrow_manual.has_lane_borrow_by_manual() ||
        !lane_borrow_manual.has_lane_borrow_dir())
    {
        return 0;
    }

    if (!lane_borrow_manual.lane_borrow_by_manual())
    {
        return 0;
    }

    const std::vector<PathData>& candidate_path_data =
            reference_line_info->GetCandidatePathData();

    if (lane_borrow_manual.lane_borrow_dir() ==
        LaneBorrowDirection::LANE_BORROW_DIRECTION_LEFT)
    {
        for (const auto& curr_path_data : candidate_path_data)
        {
            if (curr_path_data.type_ == PATH_BOUND_LANE_BORROW_LEFT)
            {
                *(reference_line_info->mutable_path_data()) = curr_path_data;
                break;
            }
        }
    }
    else
    {
        for (const auto& curr_path_data : candidate_path_data)
        {
            if (curr_path_data.type_ == PATH_BOUND_LANE_BORROW_RIGHT)
            {
                *(reference_line_info->mutable_path_data()) = curr_path_data;
                break;
            }
        }
    }

    return 0;
}

int PathAssessmentDecider::update_path_by_turtle_car_decision_lane_borrow(
        const LaneBorrowByTurtleVehicle& lane_borrow_decision,
        ReferenceLineInfo* const reference_line_info)
{
    if (lane_borrow_decision.decided_side_pass_direction_size() < 1 ||
        !lane_borrow_decision.has_lane_borrow_reason())
    {
        return 0;
    }

    const std::vector<PathData>& candidate_path_data =
            reference_line_info->GetCandidatePathData();

    // check has left lane borrow

    bool has_left_lane_borrow = false;

    for (const auto& dir : lane_borrow_decision.decided_side_pass_direction())
    {
        if (dir == LaneBorrowDirection::LANE_BORROW_DIRECTION_LEFT)
        {
            has_left_lane_borrow = true;
            break;
       }
    }

    if (has_left_lane_borrow)
    {
        for (const auto& curr_path_data : candidate_path_data)
        {
            if (curr_path_data.type_ == PATH_BOUND_LANE_BORROW_LEFT)
            {
                *(reference_line_info->mutable_path_data()) = curr_path_data;
                break;
            }
        }
    }
    // todo, future need to add lane borrow right
    // else
    // {
    //     for (const auto& curr_path_data : candidate_path_data)
    //     {
    //         if (curr_path_data.type_ == PATH_BOUND_LANE_BORROW_RIGHT)
    //         {
    //             *(reference_line_info->mutable_path_data()) = curr_path_data;
    //             break;
    //         }
    //     }
    // }

    return 0;
}

int PathAssessmentDecider::get_sl_boundary_by_delta_theta(
        SLBoundary* path_point_boundary, double ego_half_width,
        double front_to_center, double back_to_center, double delta_theta,
        const common::FrenetFramePoint& sl_path_point) const
{
    // 这里对s的精度要求不高，暂时使用近似值代替
    path_point_boundary->set_start_s(sl_path_point.s() - back_to_center);
    path_point_boundary->set_end_s(sl_path_point.s() + front_to_center);

    if (delta_theta < 0.0001 && delta_theta > -0.0001)
    {
        path_point_boundary->set_start_l(sl_path_point.l() - ego_half_width);
        path_point_boundary->set_end_l(sl_path_point.l() + ego_half_width);
    }
    else if (delta_theta > 0.0)
    {
        double cos_theta = std::cos(delta_theta) ;

        double sin_theta = std::sin(delta_theta);

        double max_l = ego_half_width * cos_theta + front_to_center * sin_theta;

        path_point_boundary->set_end_l(sl_path_point.l() + max_l);

        double min_l = ego_half_width * cos_theta + back_to_center * sin_theta;

        path_point_boundary->set_start_l(sl_path_point.l() - min_l);
    }
    else
    {
        double cos_theta = std::cos(std::fabs(delta_theta));

        double sin_theta = std::sin(std::fabs(delta_theta));

        double min_l = ego_half_width * cos_theta + front_to_center * sin_theta;

        path_point_boundary->set_start_l(sl_path_point.l() - min_l);

        double max_l = ego_half_width * cos_theta + back_to_center * sin_theta;

        path_point_boundary->set_end_l(sl_path_point.l() + max_l);
    }

    return 0;
}

}  // namespace planning
}  // namespace apollo
