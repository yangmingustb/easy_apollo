/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/tasks/deciders/speed_bounds_decider/st_boundary_mapper.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/common/math/polygon_base.h"
#include "modules/planning/constraint_checker/collision_checker.h"

namespace apollo
{
namespace planning
{
using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

#define st_boundary_mapper_time_profile (0)
#define st_boundary_mapper_debug (0)

STBoundaryMapper::STBoundaryMapper(
        const SpeedBoundsDeciderConfig& config,
        const ReferenceLine& reference_line, const PathData& path_data,
        const double planning_distance, const double planning_time,
        const std::shared_ptr<DependencyInjector>& injector) :
    speed_bounds_config_(config),
    reference_line_(reference_line),
    path_data_(path_data),
    vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()),
    planning_max_distance_(planning_distance),
    planning_max_time_(planning_time),
    injector_(injector)
{
}

Status STBoundaryMapper::ComputeSTBoundary(PathDecision* path_decision)
{
    // Sanity checks.
    CHECK_GT(planning_max_time_, 0.0);
    if (path_data_.discretized_path().size() < 2)
    {
        AERROR << "Fail to get params because of too few path points. path "
                  "points size: "
               << path_data_.discretized_path().size() << ".";

        return Status(ErrorCode::PLANNING_ERROR,
                      "Fail to get params because of too few path points");
    }

    // Go through every obstacle.
    Obstacle* stop_obstacle = nullptr;
    ObjectDecisionType stop_decision;
    double min_stop_s = std::numeric_limits<double>::max();

    // generate path
    generate_adc_polygon_path(FLAGS_nonstatic_obstacle_nudge_l_buffer,
                          path_data_.discretized_path(), adc_polygon_path_);

    for (const auto* ptr_obstacle_item : path_decision->obstacles().Items())
    {
        Obstacle* ptr_obstacle = path_decision->Find(ptr_obstacle_item->Id());

        if (ptr_obstacle == nullptr)
        {
            continue;
        }

        // If no longitudinal decision has been made, then plot it onto
        // ST-graph.
        if (!ptr_obstacle->HasLongitudinalDecision())
        {
            ComputeSTBoundary(ptr_obstacle);
            continue;
        }

        // If there is a longitudinal decision, then fine-tune boundary.
        const auto& decision = ptr_obstacle->LongitudinalDecision();
        if (decision.has_stop())
        {
            // 1. Store the closest stop fence info.
            // TODO(all): store ref. s value in stop decision; refine the code
            // then.
            common::SLPoint stop_sl_point;
            reference_line_.XYToSL(decision.stop().stop_point(),
                                   &stop_sl_point);
            const double stop_s = stop_sl_point.s();

            if (stop_s < min_stop_s)
            {
                stop_obstacle = ptr_obstacle;
                min_stop_s = stop_s;
                stop_decision = decision;
            }
        }
        else if (decision.has_follow() || decision.has_overtake() ||
                 decision.has_yield())
        {
            // 2. Depending on the longitudinal overtake/yield decision,
            //    fine-tune the upper/lower st-boundary of related obstacles.
            ComputeSTBoundaryWithDecision(ptr_obstacle, decision);
        }
        else if (!decision.has_ignore())
        {
            // 3. Ignore those unrelated obstacles.
            AWARN << "No mapping for decision: " << decision.DebugString();
        }
    }

    if (stop_obstacle)
    {
        bool success =
                ComputeSTBoundaryByStopDecision(stop_obstacle, stop_decision);
        if (!success)
        {
            const std::string msg = "Fail to MapStopDecision.";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }
    }

    return Status::OK();
}

bool STBoundaryMapper::ComputeSTBoundaryByStopDecision(
        Obstacle* stop_obstacle, const ObjectDecisionType& stop_decision) const
{
    if (!stop_decision.has_stop())
    {
        AERROR << "Must have stop decision";

        return false;
    }

    common::SLPoint ego_front_stop_sl_point;
    reference_line_.XYToSL(stop_decision.stop().stop_point(),
                           &ego_front_stop_sl_point);

    // ref line s
    // 速度规划使用的s是path s，并不是reference line s
    double rear_center_stop_s = 0.0;

    // todo: 这里不需要减去车长？
    // const double stop_ref_s =
    //         stop_sl_point.s() - vehicle_param_.front_edge_to_center();
    const double rear_center_stop_ref_s =
            ego_front_stop_sl_point.s() - vehicle_param_.front_edge_to_center();

    double path_end_point_ref_line_s =
            path_data_.frenet_frame_path().back().s();

    if (rear_center_stop_ref_s > path_end_point_ref_line_s)
    {
        rear_center_stop_s =
                path_data_.discretized_path().back().s() +
                (rear_center_stop_ref_s - path_end_point_ref_line_s);
    }
    else
    {
        // get path s
        PathPoint stop_point;
        if (!path_data_.GetPathPointWithRefS(rear_center_stop_ref_s,
                                             &stop_point))
        {
            return false;
        }
        rear_center_stop_s = stop_point.s();
    }

    // 这里生成的st bound已经包含了安全距离，所以在qp不用再加上安全距离
    const double s_min = std::fmax(0.0, rear_center_stop_s);
    const double s_max =
            std::fmax(s_min, std::fmax(planning_max_distance_, 140.0));
    // s_min, std::fmax(planning_max_distance_, reference_line_.Length()));

    // 生成静态障碍物的上下界
    // std::vector<st_point_pair>
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));
    point_pairs.emplace_back(
            STPoint(s_min, planning_max_time_),
            STPoint(s_max + speed_bounds_config_.boundary_buffer(),
                    planning_max_time_));

    auto boundary = STBoundary(point_pairs);
    boundary.SetBoundaryType(STBoundary::BoundaryType::STOP);

    // 生成stop距离
    double ego_front_to_obs_dist = std::fabs(stop_decision.stop().distance_s());

    boundary.SetCharacteristicLength(ego_front_to_obs_dist);

    boundary.set_id(stop_obstacle->Id());
    stop_obstacle->set_path_st_boundary(boundary);
    return true;
}

void STBoundaryMapper::ComputeSTBoundary(Obstacle* obstacle) const
{
    if (FLAGS_use_st_drivable_boundary)
    {
        return;
    }

#if st_boundary_mapper_time_profile
    auto time1 = std::chrono::system_clock::now();
#endif

    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;

    if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                  &upper_points, &lower_points))
    {
        return;
    }

    auto boundary = STBoundary::CreateInstance(lower_points, upper_points);
    boundary.set_id(obstacle->Id());

    // TODO(all): potential bug here.
    const auto& prev_st_boundary = obstacle->path_st_boundary();
    const auto& ref_line_st_boundary = obstacle->reference_line_st_boundary();
    if (!prev_st_boundary.IsEmpty())
    {
        boundary.SetBoundaryType(prev_st_boundary.boundary_type());
    }
    else if (!ref_line_st_boundary.IsEmpty())
    {
        boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());
    }

    obstacle->set_path_st_boundary(boundary);

#if st_boundary_mapper_time_profile
    auto time2 = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = time2 - time1;
    AINFO << "Time for ST Boundary Mapping = " << diff.count() * 1000
          << " msec, obs id: " << obstacle->PerceptionId();

    AINFO << "v: " << obstacle->speed()
          << ", path size: " << obstacle->Trajectory().trajectory_point_size();

    AINFO << "s: " << boundary.min_s()<<" , " << boundary.max_s();

#endif

    return;
}

bool STBoundaryMapper::GetOverlapBoundaryPoints(
        const std::vector<PathPoint>& path_points, const Obstacle& obstacle,
        std::vector<STPoint>* upper_points,
        std::vector<STPoint>* lower_points) const
{
    // Sanity checks.
    DCHECK(upper_points->empty());
    DCHECK(lower_points->empty());
    if (path_points.empty())
    {
        AERROR << "No points in path_data_.discretized_path().";
        return false;
    }

    const auto* planning_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_change_lane();

    double l_buffer =
            planning_status->status() == ChangeLaneStatus::IN_CHANGE_LANE
                    ? FLAGS_lane_change_obstacle_nudge_l_buffer
                    : FLAGS_nonstatic_obstacle_nudge_l_buffer;

    // Draw the given obstacle on the ST-graph.
    const auto& trajectory = obstacle.Trajectory();
    if (trajectory.trajectory_point().empty())
    {
        // For those with no predicted trajectories, just map the obstacle's
        // current position to ST-graph and always assume it's static.
        if (!obstacle.IsStatic())
        {
            AWARN << "Non-static obstacle[" << obstacle.Id()
                  << "] has NO prediction trajectory."
                  << obstacle.Perception().ShortDebugString();
        }

        int collision_index;

        bool is_collision;
        is_collision = check_path_collision_with_static_obstacle(
                &collision_index, adc_polygon_path_, obstacle);

        if (is_collision && collision_index >= 0 &&
            collision_index < path_points.size())
        {
            const PathPoint& curr_point_on_path = path_points[collision_index];

            double low_s = std::fmax(0.0, curr_point_on_path.s());

#if st_boundary_mapper_debug
            AINFO << "collision s: " << low_s;
#endif

            const Box2d& obs_box = obstacle.PerceptionBoundingBox();
            double high_s =
                    std::fmin(planning_max_distance_,
                              curr_point_on_path.s() + obs_box.length());

            // It is an unrotated rectangle appearing on the ST-graph.
            // TODO(jiacheng): reconsider the backward_distance, it might be
            // unnecessary, but forward_distance is indeed meaningful
            // though.
            lower_points->emplace_back(low_s, 0.0);
            lower_points->emplace_back(low_s, planning_max_time_);
            upper_points->emplace_back(high_s, 0.0);
            upper_points->emplace_back(high_s, planning_max_time_);
        }

        // for (const auto& curr_point_on_path : path_points)
        // {
        //     if (curr_point_on_path.s() > planning_max_distance_)
        //     {
        //         break;
        //     }

        //     const Box2d& obs_box = obstacle.PerceptionBoundingBox();
        //     if (CheckOverlap(curr_point_on_path, obs_box, l_buffer))
        //     {
        //         // If there is overlapping, then plot it on ST-graph.
        //         const double backward_distance =
        //                 -vehicle_param_.front_edge_to_center();
        //         const double forward_distance = obs_box.length();

        //         // 减去一个车长，没有意义？真实的开始碰撞的s就是curr_point_on_path.s(),
        //         // 减去一个值，是为了增加纵向安全性?
        //         double low_s = std::fmax(
        //                 0.0, curr_point_on_path.s() + backward_distance);
        //         double high_s =
        //                 std::fmin(planning_max_distance_,
        //                           curr_point_on_path.s() + forward_distance);

        //         // It is an unrotated rectangle appearing on the ST-graph.
        //         // TODO(jiacheng): reconsider the backward_distance, it might be
        //         // unnecessary, but forward_distance is indeed meaningful
        //         // though.
        //         lower_points->emplace_back(low_s, 0.0);
        //         lower_points->emplace_back(low_s, planning_max_time_);
        //         upper_points->emplace_back(high_s, 0.0);
        //         upper_points->emplace_back(high_s, planning_max_time_);
        //         break;
        //     }
        // }
    }
    else
    {
        // For those with predicted trajectories (moving obstacles):
        // 1. Subsample to reduce computation time.
        // const int default_num_point = 50;
        // DiscretizedPath discretized_path;
        // if (path_points.size() > 2 * default_num_point)
        // {
        //     // 稀疏之后，会存在漏掉的碰撞检查

        //     // 取整
        //     const int ratio = path_points.size() / default_num_point;
        //     std::vector<PathPoint> sampled_path_points;

        //     for (size_t i = 0; i < path_points.size(); ++i)
        //     {
        //         // 余数
        //         if (i % ratio == 0)
        //         {
        //             sampled_path_points.push_back(path_points[i]);
        //         }
        //     }
        //     discretized_path = DiscretizedPath(std::move(sampled_path_points));
        // }
        // else
        // {
        //     discretized_path = DiscretizedPath(path_points);
        // }

        // Box2d obs_box;

        // 和障碍物坐标系一致的polygon
        Polygon2D obs_local_polygon;

        Polygon2D obs_global_polygon;

        obs_local_polygon = obstacle.get_local_polygon();

        static constexpr double kNegtiveTimeThreshold = -1.0;

        bool is_collision;

        int first_collision_index;
        int end_collision_index;

        // 2. Go through every point of the predicted obstacle trajectory.
        for (int i = 0; i < trajectory.trajectory_point_size(); ++i)
        {
            const auto& trajectory_point = trajectory.trajectory_point(i);

            double trajectory_point_time = trajectory_point.relative_time();

            if (trajectory_point_time < kNegtiveTimeThreshold)
            {
                continue;
            }

            // obs_box = obstacle.GetBoundingBox(trajectory_point);

            obstacle.get_trajectory_point_polygon(
                    &obs_global_polygon, obs_local_polygon, trajectory_point);

            is_collision = check_path_collision_with_dynamic_obstacle_point(
                    &first_collision_index, &end_collision_index,
                    adc_polygon_path_, obs_global_polygon);

            if (is_collision && first_collision_index > -1 &&
                end_collision_index > -1)
            {
                const PathPoint& first_point_on_path =
                        path_points[first_collision_index];

                const PathPoint& end_point_on_path =
                        path_points[end_collision_index];

                double low_s = std::fmax(0.0, first_point_on_path.s() - 0.5);
                double high_s = std::fmin(planning_max_distance_,
                                          end_point_on_path.s() + 0.5);

                lower_points->emplace_back(low_s, trajectory_point_time);
                upper_points->emplace_back(high_s, trajectory_point_time);
            }

        }
    }

    // Sanity checks and return.
    DCHECK_EQ(lower_points->size(), upper_points->size());

    return (lower_points->size() > 1 && upper_points->size() > 1);
}

void STBoundaryMapper::ComputeSTBoundaryWithDecision(
        Obstacle* obstacle, const ObjectDecisionType& decision) const
{
    if (!has_dynamic_interaction_lon_decision(decision))
    {
        AERROR << "decision is " << decision.DebugString()
               << ", but it must be follow or yield or overtake.";

        return;
    }

    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;

    if (FLAGS_use_st_drivable_boundary &&
        obstacle->is_path_st_boundary_initialized())
    {
        const auto& path_st_boundary = obstacle->path_st_boundary();
        lower_points = path_st_boundary.lower_points();
        upper_points = path_st_boundary.upper_points();
    }
    else
    {
        // 只是计算st bound，并不会额外拓展
        if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                      &upper_points, &lower_points))
        {
            return;
        }
    }

    auto boundary = STBoundary::CreateInstance(lower_points, upper_points);

    // get characteristic_length and boundary_type.
    STBoundary::BoundaryType bound_type = STBoundary::BoundaryType::UNKNOWN;

    // 特征长度，follow, overtake的st bound也需要拓展？
    double characteristic_length = 0.0;

    double front_edge_to_center;
    front_edge_to_center = vehicle_param_.front_edge_to_center();

    if (decision.has_follow())
    {
        characteristic_length = std::fabs(decision.follow().distance_s());

        bound_type = STBoundary::BoundaryType::FOLLOW;

        if (decision.follow().has_following_interaction_info() &&
            decision.follow().following_interaction_info().has_intersection())
        {
            const follow_interaction_info& follow_info =
                    decision.follow().following_interaction_info();

            if (follow_info.has_end_point())
            {
                boundary.set_end_interaction_point_valid(true);
                boundary.set_end_interaction_poi(
                        follow_info.end_point().time(),
                        follow_info.end_point().s_gap());
            }
        }

    }
    else if (decision.has_yield())
    {
        characteristic_length = std::fabs(decision.yield().distance_s());

        // boundary = STBoundary::CreateInstance(lower_points, upper_points)
        //                    .ExpandLowerBoundByS(characteristic_length);

        bound_type = STBoundary::BoundaryType::YIELD;

        // yield时，可以计算st poi 关键点

        if (decision.yield().has_yield_poi() &&
            decision.yield().yield_poi().has_has_interaction())
        {
            const yield_interaction_info& yield_poi =
                    decision.yield().yield_poi();

            if (yield_poi.has_end_point())
            {
                boundary.set_end_interaction_point_valid(true);
                boundary.set_end_interaction_poi(
                        yield_poi.end_point().time(),
                        yield_poi.end_point().s_gap());
            }
        }
    }
    else if (decision.has_overtake())
    {
        characteristic_length = std::fabs(decision.overtake().distance_s());
        bound_type = STBoundary::BoundaryType::OVERTAKE;
    }
    else
    {
        DCHECK(false) << "Obj decision should be either yield or overtake: "
                      << decision.DebugString();
    }

    boundary.SetBoundaryType(bound_type);
    boundary.set_id(obstacle->Id());
    boundary.SetCharacteristicLength(characteristic_length);

    obstacle->set_path_st_boundary(boundary);
}

bool STBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double l_buffer) const
{
    // Convert reference point from center of rear axis to center of ADC.
    Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                                vehicle_param_.back_edge_to_center()) *
                                       0.5,
                               (vehicle_param_.left_edge_to_center() -
                                vehicle_param_.right_edge_to_center()) *
                                       0.5);
    ego_center_map_frame.SelfRotate(path_point.theta());
    ego_center_map_frame.set_x(ego_center_map_frame.x() + path_point.x());
    ego_center_map_frame.set_y(ego_center_map_frame.y() + path_point.y());

    // Compute the ADC bounding box.
    Box2d adc_box(ego_center_map_frame, path_point.theta(),
                  vehicle_param_.length(),
                  vehicle_param_.width() + l_buffer * 2);

    // Check whether ADC bounding box overlaps with obstacle bounding box.
    return obs_box.HasOverlap(adc_box);
}

}  // namespace planning
}  // namespace apollo
