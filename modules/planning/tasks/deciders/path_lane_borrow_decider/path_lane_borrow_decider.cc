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

#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.h"

#include <algorithm>
#include <memory>
#include <string>

#include "modules/planning/common/obstacle_blocking_analyzer.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

#include "modules/map/pnc_map/pnc_map.h"

namespace apollo
{
namespace planning
{
using apollo::common::Status;

constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;

// lane borrow related for low speed / static obstacle
#define low_speed_obs_thresh (13.8)
#define high_speed_obs_thresh (16.6)
#define time_headway_for_safe_check (5.0)
#define min_back_dist_for_lane_borrow (30.0)

// 8s/0.1s
#define lane_borrow_min_time (80)

std::string get_no_need_lane_borrow_reason(no_need_lane_borrow_reason reason)
{
    switch (reason)
    {
        case no_need_lane_borrow_reason::prior_lane_no_blocking_obs:
            return "prior_lane_no_blocking_obs";
            break;
        case no_need_lane_borrow_reason::neighbour_lane_has_obs:
            return "neighbour_lane_has_obs";
            break;
        case no_need_lane_borrow_reason::neighbour_lane_has_low_speed_obs:
            return "neighbour_lane_has_low_speed_obs";
            break;
        case no_need_lane_borrow_reason::
                no_neighbour_lane_or_break_traffic_rule:
            return "no_neighbour_lane_or_break_traffic_rule";
            break;
        case no_need_lane_borrow_reason::none:
            return "none";
            break;
        case no_need_lane_borrow_reason::neighbour_lane_has_close_obs:
            return "neighbour_lane_has_close_obs";
            break;

        default:
            break;
    }

    return "null";
}

std::string get_stop_enter_prior_lane_reason(
        stop_enter_prior_lane_reason reason)
{
    switch (reason)
    {
        case stop_enter_prior_lane_reason::lane_borrow_time_is_short:
            return "lane_borrow_time_is_short";
            break;
        case stop_enter_prior_lane_reason::prior_lane_blocking_obs:
            return "prior_lane_blocking_obs";
            break;
        case stop_enter_prior_lane_reason::prior_lane_solid_lane_line:
            return "prior_lane_solid_lane_line";
            break;
        case stop_enter_prior_lane_reason::none:
            return "none";
            break;
        default:
            break;
    }

    return "null";
}

PathLaneBorrowDecider::PathLaneBorrowDecider(
        const TaskConfig& config,
        const std::shared_ptr<DependencyInjector>& injector) :
    Decider(config, injector)
{
}

Status PathLaneBorrowDecider::Process(
        Frame* const frame, ReferenceLineInfo* const reference_line_info)
{
    // Sanity checks.
    CHECK_NOTNULL(frame);
    CHECK_NOTNULL(reference_line_info);

    // skip path_lane_borrow_decider if reused path
    if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable())
    {
        // for debug
        AINFO << "skip due to reusing path";
        return Status::OK();
    }

    // By default, don't borrow any lane.
    reference_line_info->set_is_path_lane_borrow(false);

    // Check if lane-borrowing is needed, if so, borrow lane. default value
    // is 1.
    // if (Decider::config_.path_lane_borrow_decider_config()
    //             .allow_lane_borrowing() &&
    //     IsNecessaryToBorrowLane(*frame, *reference_line_info))
    // {
    //     reference_line_info->set_is_path_lane_borrow(true);
    // }

    // hmi lane borrow
    if (update_lane_borrow_decision_by_manual(*frame, *reference_line_info))
    {
        reference_line_info->set_is_path_lane_borrow(true);
    }

    // low speed obstacle trigger lane borrow
    if (update_lane_borrow_decision_by_low_speed_obs(*frame,
                                                     *reference_line_info))
    {
        reference_line_info->set_is_path_lane_borrow(true);
    }

    // update speed limit for overtake
    double speed_error = 1.0;
    double v_limit;
    v_limit = FLAGS_planning_upper_speed_limit;

    {
        // 目前借道，都是加速超车
        const LaneBorrowStatus &lane_borrow_decision =
                injector_->planning_context()
                        ->planning_status()
                        .lane_borrow_decider();

        if (lane_borrow_decision.has_lane_borrow_reason() &&
            lane_borrow_decision.lane_borrow_reason() ==
                    LANE_BLOCKED_BY_LOW_SPEED_OBS &&
            lane_borrow_decision.has_status() &&
            lane_borrow_decision.status() == IN_LANE_BORROW)
        {
            v_limit = std::max(FLAGS_lane_borrow_max_speed,
                               FLAGS_planning_upper_speed_limit);
        }
    }

    reference_line_info->set_max_speed(v_limit);

#if debug_path_lane_borrow_decider
    auto* mutable_path_decider_status = injector_->planning_context()
                                                ->mutable_planning_status()
                                                ->mutable_lane_borrow_decider();

    AINFO << mutable_path_decider_status->DebugString();
#endif

    return Status::OK();
}

bool PathLaneBorrowDecider::update_lane_borrow_decision_by_manual(
        const Frame& frame, const ReferenceLineInfo& reference_line_info)
{
    auto* mutable_path_decider_status = injector_->planning_context()
                                                ->mutable_planning_status()
                                                ->mutable_lane_borrow_decider();

    LaneBorrowManual* lane_borrow_command =
            mutable_path_decider_status->mutable_lane_borrow_by_human();

    if (lane_borrow_command->has_lane_borrow_by_manual())
    {
        if (lane_borrow_command->lane_borrow_by_manual() &&
            lane_borrow_command->has_lane_borrow_dir())
        {
            mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(
                    true);

            mutable_path_decider_status->clear_decided_side_pass_direction();

            bool left_borrowable;
            bool right_borrowable;
            CheckLaneBorrow(reference_line_info, &left_borrowable,
                            &right_borrowable);

            if (lane_borrow_command->lane_borrow_dir() ==
                        LaneBorrowDirection::LANE_BORROW_DIRECTION_LEFT &&
                left_borrowable)
            {
                mutable_path_decider_status->add_decided_side_pass_direction(
                        LANE_BORROW_DIRECTION_LEFT);
            }
            else if (lane_borrow_command->lane_borrow_dir() ==
                             LaneBorrowDirection::LANE_BORROW_DIRECTION_RIGHT &&
                     right_borrowable)
            {
                mutable_path_decider_status->add_decided_side_pass_direction(
                        LANE_BORROW_DIRECTION_RIGHT);
            }

            mutable_path_decider_status->set_lane_borrow_reason(
                    LaneBorrowReason::LANE_BORROW_BY_HUMAN);
        }
        else
        {
            // If have been able to use self-lane for some time, then switch to
            // non-lane-borrowing.
            mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(
                    false);

            mutable_path_decider_status->clear_decided_side_pass_direction();

            mutable_path_decider_status->clear_lane_borrow_reason();

            AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";
        }
    }

    return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

bool PathLaneBorrowDecider::IsNecessaryToBorrowLane(
        const Frame& frame, const ReferenceLineInfo& reference_line_info)
{
    auto* mutable_path_decider_status = injector_->planning_context()
                                                ->mutable_planning_status()
                                                ->mutable_lane_borrow_decider();

    // If originally borrowing neighbor lane:
    if (mutable_path_decider_status->is_in_path_lane_borrow_scenario())
    {
        // 如果车道可用，并且车辆已经离开邻居车道，就关闭lane borrow scene

        bool is_adc_in_neighbor = true;



        const ReferenceLine& reference_line =
                reference_line_info.reference_line();

        const SLBoundary& sl_bound = reference_line_info.AdcSlBoundary();
        double adc_l_upper_ = sl_bound.end_l();
        double adc_l_lower_ = sl_bound.start_l();

        double adc_s = (sl_bound.start_s() + sl_bound.end_s()) / 2.0;

        // ADC's lane width.
        double lane_left_width = 0.0;
        double lane_right_width = 0.0;
        reference_line.GetLaneWidth(adc_s, &lane_left_width, &lane_right_width);

        if (adc_l_upper_ < lane_left_width && adc_l_lower_ > -lane_right_width)
        {
            is_adc_in_neighbor = false;
        }

        // 退出lane borrow state
        if (mutable_path_decider_status->able_to_use_self_lane_counter() >= 6 &&
            !is_adc_in_neighbor)
        {
            // If have been able to use self-lane for some time, then switch to
            // non-lane-borrowing.
            mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(
                    false);

            mutable_path_decider_status->clear_decided_side_pass_direction();

            AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";

            return false;
        }

        if (is_adc_in_neighbor)
        {
            if (adc_l_upper_ >= lane_left_width)
            {
                bool has_decision = false;

                size_t i;
                for (i = 0; i < mutable_path_decider_status
                                        ->decided_side_pass_direction_size();
                     i++)
                {
                    if (LaneBorrowDirection::LANE_BORROW_DIRECTION_LEFT ==
                        mutable_path_decider_status
                                ->decided_side_pass_direction(i))
                    {
                        has_decision = true;
                        break;
                    }
                }

                if (!has_decision)
                {
                    mutable_path_decider_status
                            ->add_decided_side_pass_direction(
                                    LANE_BORROW_DIRECTION_LEFT);
                }
            }

            if (adc_l_lower_ <= -lane_right_width)
            {
                bool has_decision = false;

                size_t i;
                for (i = 0; i < mutable_path_decider_status
                                        ->decided_side_pass_direction_size();
                     i++)
                {
                    if (LANE_BORROW_DIRECTION_RIGHT ==
                        mutable_path_decider_status
                                ->decided_side_pass_direction(i))
                    {
                        has_decision = true;
                        break;
                    }
                }

                if (!has_decision)
                {
                    mutable_path_decider_status
                            ->add_decided_side_pass_direction(
                                    LANE_BORROW_DIRECTION_RIGHT);
                }
            }
        }
        else
        {
            // 如果上一次循环，删除了lane borrow direction，那么本次继续更新
            if (mutable_path_decider_status->decided_side_pass_direction()
                        .empty())
            {
                // first time init decided_side_pass_direction
                bool left_borrowable;
                bool right_borrowable;
                CheckLaneBorrow(reference_line_info, &left_borrowable,
                                &right_borrowable);

                if (left_borrowable)
                {
                    mutable_path_decider_status
                            ->add_decided_side_pass_direction(
                                    LANE_BORROW_DIRECTION_LEFT);
                }

                if (right_borrowable)
                {
                    mutable_path_decider_status
                            ->add_decided_side_pass_direction(
                                    LANE_BORROW_DIRECTION_RIGHT);
                }
            }
        }
    }
    else
    {
        // If originally not borrowing neighbor lane:
        AINFO << "Blocking obstacle ID["
              << mutable_path_decider_status->front_static_obstacle_id() << "]";

        // AINFO << mutable_path_decider_status->DebugString();

        // ADC requirements check for lane-borrowing:
        if (!HasSingleReferenceLine(frame))
        {
            return false;
        }

        if (!IsWithinSidePassingSpeedADC(frame))
        {
            return false;
        }

        // Obstacle condition check for lane-borrowing:
        if (!IsBlockingObstacleFarFromIntersection(reference_line_info))
        {
            return false;
        }

        if (!IsLongTermBlockingObstacle())
        {
            return false;
        }

        // 障碍物在终点以外
        if (!IsBlockingObstacleWithinDestination(reference_line_info))
        {
            return false;
        }

        // 继续判断是否可以超车
        if (!IsSidePassableObstacle(reference_line_info))
        {
            return false;
        }

        // switch to lane-borrowing
        // set side-pass direction
        const auto& path_decider_status = injector_->planning_context()
                                                  ->planning_status()
                                                  .lane_borrow_decider();

        // 车道保持场景，清除lane borrow decision
        mutable_path_decider_status->clear_decided_side_pass_direction();
        
        if (path_decider_status.decided_side_pass_direction().empty())
        {
            // first time init decided_side_pass_direction
            bool left_borrowable;
            bool right_borrowable;
            CheckLaneBorrow(reference_line_info, &left_borrowable,
                            &right_borrowable);

            if (!left_borrowable && !right_borrowable)
            {
                mutable_path_decider_status
                        ->set_is_in_path_lane_borrow_scenario(false);
                return false;
            }
            else
            {
                mutable_path_decider_status
                        ->set_is_in_path_lane_borrow_scenario(true);

                if (left_borrowable)
                {
                    mutable_path_decider_status
                            ->add_decided_side_pass_direction(
                                    LANE_BORROW_DIRECTION_LEFT);
                }

                if (right_borrowable)
                {
                    mutable_path_decider_status
                            ->add_decided_side_pass_direction(
                                    LANE_BORROW_DIRECTION_RIGHT);
                }
            }
        }

        AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";
    }

    return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

// This function is to prevent lane-borrowing during lane-changing.
// TODO(jiacheng): depending on our needs, may allow lane-borrowing during
// lane-change.
// lane change的定义，从非优先车道进入优先车道
// 这个定义很显然有问题。未来需要修正。
bool PathLaneBorrowDecider::HasSingleReferenceLine(const Frame& frame)
{
    bool single_lane = false;
    if (frame.reference_line_info().size() == 1)
    {
        single_lane = true;
    }

    return single_lane;
}

bool PathLaneBorrowDecider::IsWithinSidePassingSpeedADC(const Frame& frame)
{
    return frame.PlanningStartPoint().v() < FLAGS_lane_borrow_max_speed;
}

bool PathLaneBorrowDecider::IsLongTermBlockingObstacle()
{
    if (injector_->planning_context()
                ->planning_status()
                .lane_borrow_decider()
                .front_static_obstacle_cycle_counter() >=
        FLAGS_long_term_blocking_obstacle_cycle_threshold)
    {
        ADEBUG << "The blocking obstacle is long-term existing.";
        return true;
    }
    else
    {
        ADEBUG << "The blocking obstacle is not long-term existing.";
        return false;
    }
}

bool PathLaneBorrowDecider::IsBlockingObstacleWithinDestination(
        const ReferenceLineInfo& reference_line_info)
{
    const auto& path_decider_status = injector_->planning_context()
                                              ->planning_status()
                                              .lane_borrow_decider();
    const std::string blocking_obstacle_id =
            path_decider_status.front_static_obstacle_id();
    if (blocking_obstacle_id.empty())
    {
        AINFO << "There is no blocking obstacle.";
        return true;
    }
    const Obstacle* blocking_obstacle =
            reference_line_info.path_decision().obstacles().Find(
                    blocking_obstacle_id);
    if (blocking_obstacle == nullptr)
    {
        AINFO << "Blocking obstacle is no longer there.";
        return true;
    }

    double blocking_obstacle_s =
            blocking_obstacle->PerceptionSLBoundary().start_s();
    double adc_end_s = reference_line_info.AdcSlBoundary().end_s();

    AINFO << "Blocking obstacle is at s = " << blocking_obstacle_s;
    AINFO << "ADC is at s = " << adc_end_s;
    AINFO << "Destination is at s = "
          << reference_line_info.SDistanceToDestination() + adc_end_s;
    
    if (blocking_obstacle_s - adc_end_s >
        reference_line_info.SDistanceToDestination())
    {
        return false;
    }
    return true;
}

bool PathLaneBorrowDecider::IsBlockingObstacleFarFromIntersection(
        const ReferenceLineInfo& reference_line_info)
{
    const auto& path_decider_status = injector_->planning_context()
                                              ->planning_status()
                                              .lane_borrow_decider();
    const std::string blocking_obstacle_id =
            path_decider_status.front_static_obstacle_id();
    if (blocking_obstacle_id.empty())
    {
        ADEBUG << "There is no blocking obstacle.";
        return true;
    }
    const Obstacle* blocking_obstacle =
            reference_line_info.path_decision().obstacles().Find(
                    blocking_obstacle_id);
    if (blocking_obstacle == nullptr)
    {
        ADEBUG << "Blocking obstacle is no longer there.";
        return true;
    }

    // Get blocking obstacle's s.
    double blocking_obstacle_s =
            blocking_obstacle->PerceptionSLBoundary().end_s();
    ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;

    // Get intersection's s and compare with threshold.
    const auto& first_encountered_overlaps =
            reference_line_info.FirstEncounteredOverlaps();
    for (const auto& overlap : first_encountered_overlaps)
    {
        ADEBUG << overlap.first << ", " << overlap.second.DebugString();
        // if (// overlap.first != ReferenceLineInfo::CLEAR_AREA &&
        // overlap.first != ReferenceLineInfo::CROSSWALK &&
        // overlap.first != ReferenceLineInfo::PNC_JUNCTION &&
        if (overlap.first != ReferenceLineInfo::SIGNAL &&
            overlap.first != ReferenceLineInfo::STOP_SIGN)
        {
            continue;
        }

        auto distance = overlap.second.start_s - blocking_obstacle_s;
        if (overlap.first == ReferenceLineInfo::SIGNAL ||
            overlap.first == ReferenceLineInfo::STOP_SIGN)
        {
            if (distance < kIntersectionClearanceDist)
            {
                ADEBUG << "Too close to signal intersection (" << distance
                       << "m); don't SIDE_PASS.";
                return false;
            }
        }
        else
        {
            if (distance < kJunctionClearanceDist)
            {
                ADEBUG << "Too close to overlap_type[" << overlap.first << "] ("
                       << distance << "m); don't SIDE_PASS";
                return false;
            }
        }
    }

    return true;
}

bool PathLaneBorrowDecider::IsSidePassableObstacle(
        const ReferenceLineInfo& reference_line_info)
{
    const auto& path_decider_status = injector_->planning_context()
                                              ->planning_status()
                                              .lane_borrow_decider();
    const std::string blocking_obstacle_id =
            path_decider_status.front_static_obstacle_id();

    if (blocking_obstacle_id.empty())
    {
        AINFO << "There is no blocking obstacle.";
        return false;
    }
    const Obstacle* blocking_obstacle =
            reference_line_info.path_decision().obstacles().Find(
                    blocking_obstacle_id);
    if (blocking_obstacle == nullptr)
    {
        AINFO << "Blocking obstacle is no longer there.";
        return false;
    }

    return IsNonmovableObstacle(reference_line_info, *blocking_obstacle);
}

void PathLaneBorrowDecider::CheckLaneBorrow(
        const ReferenceLineInfo& reference_line_info,
        bool* left_neighbor_lane_borrowable,
        bool* right_neighbor_lane_borrowable)
{
    const ReferenceLine& reference_line = reference_line_info.reference_line();

    *left_neighbor_lane_borrowable = true;
    *right_neighbor_lane_borrowable = true;

    static constexpr double kLookforwardDistance = 100.0;
    double check_s = reference_line_info.AdcSlBoundary().end_s();
    const double lookforward_distance =
            std::min(check_s + kLookforwardDistance, reference_line.Length());

    while (check_s < lookforward_distance)
    {
        auto ref_point = reference_line.GetNearestReferencePoint(check_s);
        if (ref_point.lane_waypoints().empty())
        {
            *left_neighbor_lane_borrowable = false;
            *right_neighbor_lane_borrowable = false;
            return;
        }

        const auto waypoint = ref_point.lane_waypoints().front();

        hdmap::LaneBoundaryType::Type lane_boundary_type =
                hdmap::LaneBoundaryType::UNKNOWN;

        if (*left_neighbor_lane_borrowable)
        {
            lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
            if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
                lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE)
            {
                *left_neighbor_lane_borrowable = false;
                break;
            }
            else
            {
                if (!reference_line_info.has_neighbour_lane(adc_direction::LEFT,
                                                            check_s))
                {
                    *left_neighbor_lane_borrowable = false;
                    break;
                }
            }
            ADEBUG << "s[" << check_s << "] left_lane_boundary_type["
                   << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";


        }

        if (*right_neighbor_lane_borrowable)
        {
            lane_boundary_type = hdmap::RightBoundaryType(waypoint);
            if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
                lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE)
            {
                *right_neighbor_lane_borrowable = false;
                break;
            }
            else
            {
                if (!reference_line_info.has_neighbour_lane(
                            adc_direction::RIGHT, check_s))
                {
                    *right_neighbor_lane_borrowable = false;
                    break;
                }
            }

            ADEBUG << "s[" << check_s << "] right_neighbor_lane_borrowable["
                   << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
        }
        check_s += 2.0;
    }
}

bool PathLaneBorrowDecider::has_manual_lane_borrow_decision(
        const LaneBorrowStatus& decider)
{
    if (!decider.has_lane_borrow_by_human())
    {
        return false;
    }

    if (!decider.lane_borrow_by_human().has_lane_borrow_by_manual() ||
        !decider.lane_borrow_by_human().lane_borrow_by_manual())
    {
        return false;
    }

    if (!decider.lane_borrow_by_human().has_lane_borrow_dir())
    {
        return false;
    }

    return true;
}

bool PathLaneBorrowDecider::has_lane_borrow_decision_by_turtle_car(
        const LaneBorrowStatus& decider)
{
    if (!decider.has_lane_borrow_by_turtle())
    {
        return false;
    }

    if (!decider.lane_borrow_by_turtle().has_lane_borrow_reason() ||
        decider.lane_borrow_by_turtle().decided_side_pass_direction_size() < 1)
    {
        return false;
    }

    return true;
}

bool PathLaneBorrowDecider::has_left_lane_borrow_decision_by_turtle_car(
        const LaneBorrowStatus& decider)
{
    if (!decider.has_lane_borrow_by_turtle())
    {
        return false;
    }

    if (!decider.lane_borrow_by_turtle().has_lane_borrow_reason() ||
        decider.lane_borrow_by_turtle().decided_side_pass_direction_size() < 1)
    {
        return false;
    }

    bool has_left_lane_borrow = false;

    for (const auto& dir :
         decider.lane_borrow_by_turtle().decided_side_pass_direction())
    {
        if (dir == LaneBorrowDirection::LANE_BORROW_DIRECTION_LEFT)
        {
            has_left_lane_borrow = true;
            break;
       }
    }

    return has_left_lane_borrow;
}

bool PathLaneBorrowDecider::has_left_lane_borrow_decision(
        const LaneBorrowStatus& decider)
{
    if (decider.decided_side_pass_direction_size() < 1)
    {
        return false;
    }

    bool has_left_lane_borrow = false;

    for (const auto& dir : decider.decided_side_pass_direction())
    {
        if (dir == LaneBorrowDirection::LANE_BORROW_DIRECTION_LEFT)
        {
            has_left_lane_borrow = true;
            break;
       }
    }

    return has_left_lane_borrow;
}

bool PathLaneBorrowDecider::update_lane_borrow_decision_by_low_speed_obs(
        const Frame& frame, const ReferenceLineInfo& reference_line_info)
{
    if (!FLAGS_enable_lane_borrow_for_blocking)
    {
        return false;
    }

    auto* mutable_path_decider_status = injector_->planning_context()
                                                ->mutable_planning_status()
                                                ->mutable_lane_borrow_decider();

    LaneBorrowByTurtleVehicle* lane_borrow_by_turtle_car =
            mutable_path_decider_status->mutable_lane_borrow_by_turtle();

    hdmap::lane_position_priority adc_position;

    bool is_adc_in_neighbor = true;

    const ReferenceLine& reference_line = reference_line_info.reference_line();

    const SLBoundary& sl_bound = reference_line_info.AdcSlBoundary();
    double adc_l_upper_ = sl_bound.end_l();
    double adc_l_lower_ = sl_bound.start_l();

    double adc_l_center = (adc_l_upper_ + adc_l_lower_) / 2.0;

    double adc_s = (sl_bound.start_s() + sl_bound.end_s()) / 2.0;
    double adc_end_s = sl_bound.end_s();

    // ADC's lane width.
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    reference_line.GetLaneWidth(adc_s, &lane_left_width, &lane_right_width);

    if (adc_l_upper_ > lane_left_width)
    {
        adc_position = hdmap::lane_position_priority::left_forward;
    }
    else if (adc_l_lower_ < -lane_right_width)
    {
        adc_position = hdmap::lane_position_priority::right_forward;
    }
    else
    {
        adc_position = hdmap::lane_position_priority::priority_lane;
    }

    double left_neighbour_lane_width = 0.0;

    no_need_lane_borrow_reason no_need_lane_borrow_type =
            no_need_lane_borrow_reason::none;

    // 检查低速障碍物
    bool is_prior_lane_low_speed_obs = false;

    // 检查交规，可以借道
    bool left_borrow_by_traffic_rule = false;
    bool right_borrow_by_traffic_rule = false;

    // 检查安全性，可以借道
    bool left_borrow_by_safe = false;
    bool right_borrow_by_safe = false;

    double prior_lane_close_obs_s = 200000.0;
    double prior_lane_close_obs_v = 100.0;

    double left_lane_close_obs_s = 200000.0;
    double left_lane_close_obs_v = 100.0;

    bool need_to_enter_prior_lane = false;

    // check should lane borrow
    if (!mutable_path_decider_status->is_in_path_lane_borrow_scenario() &&
        adc_position == hdmap::lane_position_priority::priority_lane)
    {
        const auto& indexed_obstacles =
                reference_line_info.path_decision().obstacles();



        for (const auto* obstacle : indexed_obstacles.Items())
        {
            if (obstacle->IsVirtual())
            {
                continue;
            }

            const auto& obstacle_sl = obstacle->PerceptionSLBoundary();

            if (obstacle_sl.start_s() > adc_end_s + 100.0)
            {
                continue;
            }

            if (obstacle_sl.end_s() < adc_s)
            {
                continue;
            }

            if (obstacle_sl.start_l() > lane_left_width)
            {
                continue;
            }

            if (obstacle_sl.end_l() < -lane_right_width)
            {
                continue;
            }

            if (obstacle_sl.start_s() - adc_end_s >
                reference_line_info.SDistanceToDestination())
            {
                continue;
            }

            if (obstacle_sl.start_s() > adc_end_s)
            {
                if (obstacle_sl.start_s() < prior_lane_close_obs_s)
                {
                    prior_lane_close_obs_s = obstacle_sl.start_s();
                    prior_lane_close_obs_v = obstacle->speed();
                }
            }
        }

        if (prior_lane_close_obs_v < low_speed_obs_thresh)
        {
            is_prior_lane_low_speed_obs = true;
        }

        do
        {
            if (!is_prior_lane_low_speed_obs)
            {
                no_need_lane_borrow_type =
                        no_need_lane_borrow_reason::prior_lane_no_blocking_obs;
                break;
            }

            // switch to lane-borrowing
            // set side-pass direction
            const auto& path_decider_status = injector_->planning_context()
                                                      ->planning_status()
                                                      .lane_borrow_decider();

            // 车道保持场景，清除lane borrow decision
            mutable_path_decider_status->clear_decided_side_pass_direction();

            // first time init decided_side_pass_direction

            CheckLaneBorrow(reference_line_info, &left_borrow_by_traffic_rule,
                            &right_borrow_by_traffic_rule);

            if (!left_borrow_by_traffic_rule)
            {
                mutable_path_decider_status
                        ->set_is_in_path_lane_borrow_scenario(false);

                no_need_lane_borrow_type = no_need_lane_borrow_reason::
                        no_neighbour_lane_or_break_traffic_rule;
                break;
            }

            hdmap::Id neighbor_lane_id;
            if (left_borrow_by_traffic_rule)
            {
                // Borrowing left neighbor lane.
                reference_line_info.GetNeighborLaneInfo(
                        ReferenceLineInfo::LaneType::LeftForward, adc_s,
                        &neighbor_lane_id, &left_neighbour_lane_width);
            }

            // update left lane l range
            double left_neighbour_lane_start_l;
            double left_neighbour_lane_end_l;

            if (left_neighbour_lane_width < 1.0)
            {
                no_need_lane_borrow_type = no_need_lane_borrow_reason::
                        no_neighbour_lane_or_break_traffic_rule;
                break;
            }

            left_neighbour_lane_start_l = lane_left_width;

            left_neighbour_lane_end_l =
                    left_neighbour_lane_width + lane_left_width;

            // 继续检查safety
            double obs_time_headway;

            if (left_borrow_by_traffic_rule)
            {
                for (const auto* obstacle : indexed_obstacles.Items())
                {
                    if (obstacle->IsVirtual())
                    {
                        continue;
                    }

                    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();

                    // too far
                    if (obstacle_sl.start_s() > adc_end_s + 100.0)
                    {
                        continue;
                    }

                    // back too far
                    if (obstacle_sl.end_s() < adc_end_s - 150.0)
                    {
                        continue;
                    }

                    // 是否属于邻居车道
                    if (obstacle_sl.start_l() > left_neighbour_lane_end_l)
                    {
                        continue;
                    }

                    if (obstacle_sl.end_l() < left_neighbour_lane_start_l)
                    {
                        continue;
                    }

                    // 检查属于prior lane还是neighbour lane
                    if (obstacle_sl.end_l() > left_neighbour_lane_start_l)
                    {
                        // obs跨越两个车道，但是占用邻居车道小于1m
                        if (obstacle_sl.start_l() <
                                    left_neighbour_lane_start_l &&
                            (obstacle_sl.end_l() -
                             left_neighbour_lane_start_l) < 1.0)
                        {
                            continue;
                        }
                    }

                    // 换道区间内，有obstacle

                    obs_time_headway =
                            obstacle->speed() * time_headway_for_safe_check;

                    obs_time_headway = std::max(obs_time_headway,
                                                min_back_dist_for_lane_borrow);

                    // back
                    if (obstacle_sl.end_s() < adc_s)
                    {
                        if (obstacle_sl.end_s() + obs_time_headway > adc_s)
                        {
                            left_borrow_by_safe = false;

                            no_need_lane_borrow_type =
                                    no_need_lane_borrow_reason::
                                            neighbour_lane_has_obs;
                            break;
                        }
                    }
                    // front
                    else
                    {
                        if (obstacle_sl.start_s() < adc_end_s + 30.0)
                        {
                            left_borrow_by_safe = false;

                            no_need_lane_borrow_type =
                                    no_need_lane_borrow_reason::
                                            neighbour_lane_has_obs;
                            break;
                        }
                    }

                    // 检查左侧车道，前方障碍物速度
                    if (obstacle_sl.start_s() > adc_end_s)
                    {
                        if (obstacle_sl.start_s() < left_lane_close_obs_s)
                        {
                            left_lane_close_obs_s = obstacle_sl.start_s();
                            left_lane_close_obs_v = obstacle->speed();
                        }
                    }

                    if (left_lane_close_obs_s < adc_end_s + 100.0 &&
                        left_lane_close_obs_v < high_speed_obs_thresh)
                    {
                        no_need_lane_borrow_type = no_need_lane_borrow_reason::
                                neighbour_lane_has_low_speed_obs;

                        left_borrow_by_safe = false;
                        break;
                    }
                }

            }

            if (left_borrow_by_traffic_rule &&
                (no_need_lane_borrow_type == no_need_lane_borrow_reason::none) &&
                (left_lane_close_obs_v > high_speed_obs_thresh) &&
                (left_lane_close_obs_s > prior_lane_close_obs_s))
            {
                left_borrow_by_safe = true;

                mutable_path_decider_status
                        ->set_is_in_path_lane_borrow_scenario(true);

                mutable_path_decider_status->add_decided_side_pass_direction(
                        LANE_BORROW_DIRECTION_LEFT);

                mutable_path_decider_status->set_lane_borrow_reason(
                        LANE_BLOCKED_BY_LOW_SPEED_OBS);

                mutable_path_decider_status->set_status(IN_LANE_BORROW);

                // update lane borrow by turtle car
                lane_borrow_by_turtle_car->Clear();

                lane_borrow_by_turtle_car->set_lane_borrow_reason(
                        LANE_BLOCKED_BY_LOW_SPEED_OBS);

                lane_borrow_by_turtle_car->add_decided_side_pass_direction(
                        LANE_BORROW_DIRECTION_LEFT);
            }

            if (left_lane_close_obs_s <= prior_lane_close_obs_s)
            {
                no_need_lane_borrow_type = no_need_lane_borrow_reason::
                        neighbour_lane_has_close_obs;
            }

            // todo
            // if (right_borrow_by_traffic_rule)
            // {
            //     mutable_path_decider_status->add_decided_side_pass_direction(
            //             LANE_BORROW_DIRECTION_RIGHT);
            // }


        } while (0);

        // ego 中心回归到lane center，才算lane borrow 结束
        if (adc_l_center < 0.2 && adc_l_center > -0.2)
        {
            if (!mutable_path_decider_status->is_in_path_lane_borrow_scenario())
            {
                mutable_path_decider_status->set_status(LANE_BORROW_FINISH);
            }
        }

        return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
    }

    // check turtle car lane borrow scene
    if (mutable_path_decider_status->has_is_in_path_lane_borrow_scenario() &&
        mutable_path_decider_status->is_in_path_lane_borrow_scenario())
    {
        // 存在lane borrow decision
        if (lane_borrow_by_turtle_car->has_lane_borrow_reason() &&
            lane_borrow_by_turtle_car->decided_side_pass_direction_size() > 0)
        {
            // update lane borrow time
            int32_t lane_borrow_time = 0;

            stop_enter_prior_lane_reason stop_enter_prior_lane =
                    stop_enter_prior_lane_reason::none;

            if (lane_borrow_by_turtle_car->has_lane_borrow_counter())
            {
                lane_borrow_time =
                        lane_borrow_by_turtle_car->lane_borrow_counter();
            }
            else
            {
                lane_borrow_time = 0;
            }

            hdmap::lane_position_priority adc_center_position;

            if (adc_l_center > lane_left_width)
            {
                adc_center_position =
                        hdmap::lane_position_priority::left_forward;
            }
            else if (adc_l_center < -lane_right_width)
            {
                adc_center_position =
                        hdmap::lane_position_priority::right_forward;
            }
            else
            {
                adc_center_position =
                        hdmap::lane_position_priority::priority_lane;
            }

            if (adc_center_position !=
                hdmap::lane_position_priority::priority_lane)
            {
                lane_borrow_time++;
            }

            lane_borrow_time = std::min(lane_borrow_time, 10000);

            lane_borrow_by_turtle_car->set_lane_borrow_counter(
                    lane_borrow_time);

            // check 是否可以回归到优先车道

            do
            {
                // check time
                if (lane_borrow_time < lane_borrow_min_time)
                {
                    need_to_enter_prior_lane = false;

                    stop_enter_prior_lane = stop_enter_prior_lane_reason::
                            lane_borrow_time_is_short;
                    break;
                }

                const auto& indexed_obstacles =
                        reference_line_info.path_decision().obstacles();

                double obs_time_headway;

                // check prior lane
                for (const auto* obstacle : indexed_obstacles.Items())
                {
                    if (obstacle->IsVirtual())
                    {
                        continue;
                    }

                    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();

                    // too far
                    if (obstacle_sl.start_s() > adc_end_s + 100.0)
                    {
                        continue;
                    }

                    // back too far
                    if (obstacle_sl.end_s() < adc_end_s - 150.0)
                    {
                        continue;
                    }

                    // 不属于prior lane
                    if (obstacle_sl.start_l() > lane_left_width)
                    {
                        continue;
                    }

                    if (obstacle_sl.end_l() < -lane_right_width)
                    {
                        continue;
                    }

                    // destination outside
                    if (obstacle_sl.start_s() - adc_end_s >
                        reference_line_info.SDistanceToDestination())
                    {
                        continue;
                    }

                    // 检查属于prior lane还是neighbour lane
                    if (obstacle_sl.start_l() < lane_left_width)
                    {
                        // obs跨越两个车道，但是占用prior车道小于1m
                        if (obstacle_sl.end_l() > lane_left_width &&
                            obstacle_sl.start_l() > (lane_left_width - 1.0))
                        {
                            continue;
                        }
                    }

                    obs_time_headway =
                            obstacle->speed() * time_headway_for_safe_check;

                    obs_time_headway = std::max(obs_time_headway,
                                                min_back_dist_for_lane_borrow);

                    // back
                    if (obstacle_sl.end_s() < adc_s)
                    {
                        //static
                        if (obstacle->speed() < 0.2)
                        {
                            if (obstacle_sl.end_s() + 15.0 > adc_s)
                            {
                                stop_enter_prior_lane =
                                        stop_enter_prior_lane_reason::
                                                prior_lane_blocking_obs;

                                AINFO << "prior lane has obstacle, s: "
                                      << obstacle_sl.start_s() << ", "
                                      << obstacle_sl.end_s()
                                      << "adc s: " << adc_s;

                                need_to_enter_prior_lane = false;

                                break;
                            }
                        }
                        // dynamic
                        else if (obstacle_sl.end_s() + obs_time_headway > adc_s)
                        {
                            stop_enter_prior_lane =
                                    stop_enter_prior_lane_reason::
                                            prior_lane_blocking_obs;

                            AINFO << "prior lane has obstacle, s: "
                                  << obstacle_sl.start_s() << ", "
                                  << obstacle_sl.end_s() << "adc s: " << adc_s;

                            need_to_enter_prior_lane = false;

                            break;
                        }
                    }
                    // front
                    else
                    {
                        if (obstacle_sl.start_s() < (adc_end_s + 50.0))
                        {
                            stop_enter_prior_lane =
                                    stop_enter_prior_lane_reason::
                                            prior_lane_blocking_obs;

                            AINFO << "prior lane has obstacle, s: "
                                  << obstacle_sl.start_s() << ", "
                                  << obstacle_sl.end_s() << "adc s: " << adc_s;

                            need_to_enter_prior_lane = false;

                            break;
                        }
                    }

                    // 更新区间前方obs特征
                    if (obstacle_sl.start_s() > adc_end_s)
                    {
                        if (obstacle_sl.start_s() < prior_lane_close_obs_s)
                        {
                            prior_lane_close_obs_s = obstacle_sl.start_s();
                            prior_lane_close_obs_v = obstacle->speed();
                        }
                    }
                }

                // check obs
                if (stop_enter_prior_lane == stop_enter_prior_lane_reason::none)
                {
                    if (prior_lane_close_obs_s > (50.0 + adc_end_s) &&
                        prior_lane_close_obs_v > high_speed_obs_thresh)
                    {
                        need_to_enter_prior_lane = true;
                        break;
                    }
                }

            } while (0);

            AINFO << "need_to_enter_prior_lane: " << need_to_enter_prior_lane
                  << " prior_lane_close_obs_s " << prior_lane_close_obs_s
                  << " prior_lane_close_obs_v " << prior_lane_close_obs_v
                  << get_stop_enter_prior_lane_reason(stop_enter_prior_lane)
                  << " adc_end_s " << adc_end_s;

            if (need_to_enter_prior_lane)
            {
                // If have been able to use self-lane for some time, then switch
                // to
                // non-lane-borrowing.
                mutable_path_decider_status
                        ->set_is_in_path_lane_borrow_scenario(false);

                mutable_path_decider_status
                        ->clear_decided_side_pass_direction();

                mutable_path_decider_status->clear_lane_borrow_reason();

                lane_borrow_by_turtle_car->Clear();
            }
            else
            {
                // check
                bool has_left_lane_borrow = false;

                const auto& path_decider_status =
                        injector_->planning_context()
                                ->planning_status()
                                .lane_borrow_decider();
                has_left_lane_borrow =
                        has_left_lane_borrow_decision(path_decider_status);

                for (const auto& dir :
                     lane_borrow_by_turtle_car->decided_side_pass_direction())
                {
                    if (dir == LANE_BORROW_DIRECTION_LEFT &&
                        !has_left_lane_borrow)
                    {
                        mutable_path_decider_status
                                ->add_decided_side_pass_direction(
                                        LANE_BORROW_DIRECTION_LEFT);
                    }
                }

                mutable_path_decider_status->set_lane_borrow_reason(
                        LANE_BLOCKED_BY_LOW_SPEED_OBS);

            }

            mutable_path_decider_status->set_status(IN_LANE_BORROW);
        }
    }

    AINFO << "can not LANE-BORROW reason: "
          << get_no_need_lane_borrow_reason(no_need_lane_borrow_type)
          << " is_low_speed_obs " << is_prior_lane_low_speed_obs
          << "  left_borrow_by_safe " << left_borrow_by_safe
          << " left close_obs_s " << left_lane_close_obs_s
          << "left close_obs_v " << left_lane_close_obs_v
          << " left_borrow_by_traffic_rule " << left_borrow_by_traffic_rule
          << " prior_lane_close_obs_s " << prior_lane_close_obs_s
          << " left_neighbour_lane_width " << left_neighbour_lane_width
          << " high_speed_obs_thresh " << high_speed_obs_thresh;

    return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

}  // namespace planning
}  // namespace apollo
