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

#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"

#include <algorithm>
#include <limits>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/decision.pb.h"

namespace apollo
{
namespace planning
{
using apollo::common::Status;

#define debug_speed_limit (0)

SpeedLimitDecider::SpeedLimitDecider(const SpeedBoundsDeciderConfig& config,
                                     const ReferenceLine& reference_line,
                                     const PathData& path_data) :
    speed_bounds_config_(config),
    reference_line_(reference_line),
    path_data_(path_data),
    vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param())
{
}

Status SpeedLimitDecider::GetSpeedLimits(
        const IndexedList<std::string, Obstacle>& obstacles,
        SpeedLimit* const speed_limit_data,
        const double speed_limit_by_scenario) const
{
    CHECK_NOTNULL(speed_limit_data);

    const auto& discretized_path = path_data_.discretized_path();
    const auto& frenet_path = path_data_.frenet_frame_path();

    bool is_curve_lane = false;

    double curve_lane_s = 1000.0;

    for (uint32_t i = 0; i < discretized_path.size(); ++i)
    {
        const double path_s = discretized_path.at(i).s();
        const double reference_line_s = frenet_path.at(i).s();
        if (reference_line_s > reference_line_.Length())
        {
            AWARN << "path w.r.t. reference line at [" << reference_line_s
                  << "] is LARGER than reference_line_ length ["
                  << reference_line_.Length()
                  << "]. Please debug before proceeding.";
            break;
        }

        // (1) speed limit from map
        double speed_limit_from_reference_line =
                reference_line_.GetSpeedLimitFromS(reference_line_s,
                                                   speed_limit_by_scenario);

        // 一直搜索弯道
        if (enable_brake_prepare_before_curve_lane)
        {
            if (!is_curve_lane)
            {
                is_curve_lane = reference_line_.has_curve_lane(
                        reference_line_s +
                        brake_prepare_dist_before_curve_lane);

                if (is_curve_lane)
                {
                    speed_limit_from_reference_line =
                            std::min(speed_limit_from_reference_line,
                                     curve_lane_max_speed);

                    curve_lane_s = reference_line_s +
                                   brake_prepare_dist_before_curve_lane;
                }
            }
            else
            {
                if (reference_line_s < curve_lane_s)
                {
                    speed_limit_from_reference_line =
                            std::min(speed_limit_from_reference_line,
                                     curve_lane_max_speed);
                }
            }
        }

        // (2) speed limit from path curvature
        //  -- 2.1: limit by centripetal force (acceleration)
        const double speed_limit_from_centripetal_acc = std::sqrt(
                speed_bounds_config_.max_centric_acceleration_limit() /
                std::fmax(std::fabs(discretized_path.at(i).kappa()),
                          speed_bounds_config_.minimal_kappa()));

        // (3) speed limit from nudge obstacles
        // TODO(all): in future, expand the speed limit not only to obstacles
        // with nudge decisions.
        double speed_limit_from_nearby_obstacles =
                std::numeric_limits<double>::max();

        // 小于1米，安全性限速
        const double collision_safety_range =
                speed_bounds_config_.collision_safety_range();

        for (const auto* ptr_obstacle : obstacles.Items())
        {
            if (ptr_obstacle->IsVirtual())
            {
                continue;
            }
            if (!ptr_obstacle->LateralDecision().has_nudge())
            {
                continue;
            }

            /* ref line:
             * -------------------------------
             *    start_s   end_s
             * ------|  adc   |---------------
             * ------------|  obstacle |------
             */

            // TODO(all): potential problem here;
            // frenet and cartesian coordinates are mixed.
            const double vehicle_front_s =
                    reference_line_s + vehicle_param_.front_edge_to_center();
            const double vehicle_back_s =
                    reference_line_s - vehicle_param_.back_edge_to_center();
            const double obstacle_front_s =
                    ptr_obstacle->PerceptionSLBoundary().end_s();
            const double obstacle_back_s =
                    ptr_obstacle->PerceptionSLBoundary().start_s();

            if (vehicle_front_s < obstacle_back_s ||
                vehicle_back_s > obstacle_front_s)
            {
                continue;
            }

            const auto& nudge_decision =
                    ptr_obstacle->LateralDecision().nudge();

            // Please notice the differences between adc_l and frenet_point_l
            const double frenet_point_l = frenet_path.at(i).l();

            // obstacle is on the right of ego vehicle (at path point i)
            bool is_close_on_left =
                    (nudge_decision.type() == ObjectNudge::LEFT_NUDGE) &&
                    (frenet_point_l - vehicle_param_.right_edge_to_center() -
                             collision_safety_range <
                     ptr_obstacle->PerceptionSLBoundary().end_l());

            // obstacle is on the left of ego vehicle (at path point i)
            bool is_close_on_right =
                    (nudge_decision.type() == ObjectNudge::RIGHT_NUDGE) &&
                    (ptr_obstacle->PerceptionSLBoundary().start_l() -
                             collision_safety_range <
                     frenet_point_l + vehicle_param_.left_edge_to_center());

            // TODO(all): dynamic obstacles do not have nudge decision
            if (is_close_on_left || is_close_on_right)
            {
                double nudge_speed_ratio = 1.0;
                if (ptr_obstacle->IsStatic())
                {
                    nudge_speed_ratio =
                            speed_bounds_config_.static_obs_nudge_speed_ratio();
                }
                else
                {
                    nudge_speed_ratio =
                            speed_bounds_config_
                                    .dynamic_obs_nudge_speed_ratio();
                }

                // TODO:
                // 安全性限速的值，应该和车辆自身速度有关，而不是路网限速*系数？
                // 或者这里单独设定限速值，行人附近限速10km/h，护栏附近限速20km/h
                speed_limit_from_nearby_obstacles =
                        nudge_speed_ratio * speed_limit_from_reference_line;
                break;
            }
        }

        double curr_speed_limit = 0.0;

        // 1
        if (FLAGS_enable_nudge_slowdown)
        {
            curr_speed_limit =
                    std::fmax(speed_bounds_config_.lowest_speed(),
                              std::min({speed_limit_from_reference_line,
                                        speed_limit_from_centripetal_acc,
                                        speed_limit_from_nearby_obstacles}));
        }
        else
        {
            curr_speed_limit =
                    std::fmax(speed_bounds_config_.lowest_speed(),
                              std::min({speed_limit_from_reference_line,
                                        speed_limit_from_centripetal_acc}));
        }
        speed_limit_data->AppendSpeedLimit(path_s, curr_speed_limit);
    }

#if debug_speed_limit
    AINFO << "origin v limit";
    speed_limit_data->debug_string();
#endif

    return Status::OK();
}

int SpeedLimitDecider::smooth_speed_limit(SpeedLimit* speed_limit_data,
                                          double adc_speed)
{
	// constant acc 
    // v ^ 2 = v_0 ^ 2 + 2 * acc * s;

    // 使用当前车速去平滑限速
    double k_smooth_speed_limit_dec = -2.0;
	double k_speed_limit_buffer = 0.5;

    const std::vector<std::pair<double, double>>& speed_limit_points =
            speed_limit_data->speed_limit_points();

    double stop_s = -adc_speed * adc_speed / 2.0 / k_smooth_speed_limit_dec;

    double smooth_v;
    for (size_t i = 0; i < speed_limit_points.size(); i++)
	{
        const std::pair<double, double>& point = speed_limit_points[i];

        smooth_v = adc_speed * adc_speed +
                   2 * k_smooth_speed_limit_dec * point.first;

        smooth_v = std::max(0.0, std::sqrt(smooth_v));

        smooth_v = std::max(speed_bounds_config_.lowest_speed(), smooth_v);

        if (point.second <= smooth_v)
        {
            speed_limit_data->set_speed_limit_by_index(
                    i, smooth_v + k_speed_limit_buffer);
        }

        if (point.first > stop_s)
        {
            break;
        }
    }
	
#if debug_speed_limit
    AINFO << "smooth v limit";
    speed_limit_data->debug_string();
#endif
    return 0;
}

}  // namespace planning
}  // namespace apollo
