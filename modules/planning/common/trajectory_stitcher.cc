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

#include "modules/planning/common/trajectory_stitcher.h"

#include <algorithm>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_model/vehicle_model.h"
#include "modules/planning/common/planning_gflags.h"
#include <iomanip>

namespace apollo
{
namespace planning
{
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleModel;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;

#define replan_by_lon_offset (1)

TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
        const double planning_cycle_time, const VehicleState& vehicle_state)
{
    TrajectoryPoint point;
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_x(vehicle_state.x());
    point.mutable_path_point()->set_y(vehicle_state.y());
    point.mutable_path_point()->set_z(vehicle_state.z());
    point.mutable_path_point()->set_theta(vehicle_state.heading());
    point.mutable_path_point()->set_kappa(vehicle_state.kappa());
    point.set_v(vehicle_state.linear_velocity());
    point.set_a(vehicle_state.linear_acceleration());
    point.set_relative_time(planning_cycle_time);
    return point;
}

std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
        const double planning_cycle_time, const VehicleState& vehicle_state)
{
    TrajectoryPoint reinit_point;
    static constexpr double kEpsilon_v = 0.1;
    static constexpr double kEpsilon_a = 0.4;

    // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided
    if (std::fabs(vehicle_state.linear_velocity()) < kEpsilon_v &&
        std::fabs(vehicle_state.linear_acceleration()) < kEpsilon_a)
    {
        reinit_point = ComputeTrajectoryPointFromVehicleState(
                planning_cycle_time, vehicle_state);
    }
    else
    {
        VehicleState predicted_vehicle_state;
        predicted_vehicle_state =
                VehicleModel::Predict(planning_cycle_time, vehicle_state);
        reinit_point = ComputeTrajectoryPointFromVehicleState(
                planning_cycle_time, predicted_vehicle_state);
    }

    return std::vector<TrajectoryPoint>(1, reinit_point);
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
        const double x_diff, const double y_diff, const double theta_diff,
        PublishableTrajectory* prev_trajectory)
{
    if (!prev_trajectory)
    {
        return;
    }

    // R^-1
    double cos_theta = std::cos(theta_diff);
    double sin_theta = -std::sin(theta_diff);

    // -R^-1 * t
    auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
    auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

    std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                  [&cos_theta, &sin_theta, &tx, &ty,
                   &theta_diff](common::TrajectoryPoint& p) {
                      auto x = p.path_point().x();
                      auto y = p.path_point().y();
                      auto theta = p.path_point().theta();

                      auto x_new = cos_theta * x - sin_theta * y + tx;
                      auto y_new = sin_theta * x + cos_theta * y + ty;
                      auto theta_new =
                              common::math::NormalizeAngle(theta - theta_diff);

                      p.mutable_path_point()->set_x(x_new);
                      p.mutable_path_point()->set_y(y_new);
                      p.mutable_path_point()->set_theta(theta_new);
                  });
}

/* Planning from current vehicle state if:
1. the auto-driving mode is off
(or) 2. we don't have the trajectory from last planning cycle
(or) 3. the position deviation from actual and target is too high
*/
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
        const VehicleState& vehicle_state, const double current_timestamp,
        const double planning_cycle_time, const size_t preserved_points_num,
        const bool replan_by_offset,
        const PublishableTrajectory* prev_trajectory,
        std::string* replan_reason)
{
    if (!FLAGS_enable_trajectory_stitcher)
    {
        *replan_reason = "stitch is disabled by gflag.";
        return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                vehicle_state);
    }

    if (!prev_trajectory)
    {
        *replan_reason = "replan for no previous trajectory.";
        return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                vehicle_state);
    }

    if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE)
    {
        *replan_reason = "replan for manual mode.";
        return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                vehicle_state);
    }

    size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

    if (prev_trajectory_size == 0)
    {
        ADEBUG << "Projected trajectory at time ["
               << prev_trajectory->header_time()
               << "] size is zero! Previous planning not exist or failed. Use "
                  "origin car status instead.";
        *replan_reason = "replan for empty previous trajectory.";
        return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                vehicle_state);
    }

    // 相对于上一帧轨迹的时间.轨迹时间是planning模块的起始时间
    const double veh_rel_time =
            current_timestamp - prev_trajectory->header_time();

    // AINFO << std::fixed << std::setprecision(3) << current_timestamp;
    // AINFO << std::fixed << std::setprecision(3)
    //       << prev_trajectory->header_time();
    // AINFO << std::fixed << std::setprecision(3) << veh_rel_time;

    size_t time_matched_index =
            prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

    // 历史轨迹起点是当前时刻0.1秒后的预测点，下一次规划开始时，
    // 投影点可能就是上一个轨迹的起点，因为上一次预测很准，这样就导致不停重规划，这是不期望的，
    // 总是从车辆状态开始规划，会引起方向盘振荡。
    // 所以，关闭这个条件？
    double time_error = 0.1;
    if (time_matched_index == 0 &&
        veh_rel_time + time_error <
                prev_trajectory->StartPoint().relative_time())
    {
        AWARN << "current time smaller than the previous trajectory's first "
                 "time";

        AINFO << veh_rel_time << " , "
              << prev_trajectory->StartPoint().relative_time();

        *replan_reason = "replan for current time smaller than the previous "
                         "trajectory's first time.";
        return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                vehicle_state);
    }

    if (time_matched_index + 1 >= prev_trajectory_size)
    {
        AWARN << "current time beyond the previous trajectory's last time";
        *replan_reason = "replan for current time beyond the previous "
                         "trajectory's last time";
        return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                vehicle_state);
    }

    // 时间戳对应的轨迹点
    auto time_matched_point = prev_trajectory->TrajectoryPointAt(
            static_cast<uint32_t>(time_matched_index));

    if (!time_matched_point.has_path_point())
    {
        *replan_reason = "replan for previous trajectory missed path point";
        return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                vehicle_state);
    }

    // 找到位置匹配点。最近点就是匹配点.当速度曲线为0时，position_matched_index这个值
    // 可能很大
    size_t position_matched_index =
            prev_trajectory->QueryNearestPointWithBuffer(
                    {vehicle_state.x(), vehicle_state.y()}, 1.0e-6);

    // 相对于轨迹的sl.
    // position_matched_index
    // 这个值在停车状态下，可能很大。因为t在增长，s不变，如果根据位置点得到轨迹索引，
    // 那么生成的轨迹会越来越长
    auto frenet_sd = ComputePositionProjection(
            vehicle_state.x(), vehicle_state.y(),
            prev_trajectory->TrajectoryPointAt(
                    static_cast<uint32_t>(position_matched_index)));

    if (replan_by_offset)
    {
        auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
        auto lat_diff = frenet_sd.second;

        ADEBUG << "Control lateral diff: " << lat_diff
               << ", longitudinal diff: " << lon_diff;

        // 0.5 meter
        if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold)
        // if (0)
        {
            const std::string msg = absl::StrCat(
                    "the distance between matched point and actual "
                    "position is "
                    "too large. Replan is triggered. lat_diff = ",
                    lat_diff);
            AINFO << msg;
            *replan_reason = msg;

            return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                    vehicle_state);
        }

        // 2.5米。
        if (replan_by_lon_offset)
        {
            if (std::fabs(lon_diff) >
                FLAGS_replan_longitudinal_distance_threshold)
            {
                const std::string msg = absl::StrCat(
                        "the distance between matched point and actual "
                        "position is "
                        "too large. Replan is triggered. lon_diff = ",
                        lon_diff);

                *replan_reason = msg;
                AINFO << msg;

#if debug_traj_stitch
                
                AINFO << "veh speed: " << vehicle_state.linear_velocity();
                AINFO << "Position matched index: " << position_matched_index;
                AINFO << "Time matched index: " << time_matched_index;

                for (size_t i = 0; i < prev_trajectory->size(); i++)
                {
                    AINFO << "path point id: " << i;
                    AINFO << prev_trajectory->at(i).DebugString();

                    if (i > position_matched_index && i > time_matched_index)
                    {
                        break;
                    }
                }

#endif

                return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                        vehicle_state);
            }
        }
    }

    // 基本在200毫秒左右
    double forward_rel_time = veh_rel_time + planning_cycle_time;

    // 如果车辆已经运动起来，那么这个轨迹点选择是合理的。
    // 如果车辆一直没有动，但是轨迹的速度一直在增长，会让这个值越来越大。
    size_t forward_time_index =
            prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

    // stitch traj start
    auto start_pt_index = std::min(time_matched_index, position_matched_index);

    int stitch_start_point;
    stitch_start_point = std::max(
            0, static_cast<int>(start_pt_index - preserved_points_num));

    // stitch traj end
    int stitch_end_point;

    // 如果底盘的速度很低，轨迹起点依然选择车辆后轴附近
    if (vehicle_state.linear_velocity() < 0.02)
    {
        stitch_end_point = std::min(forward_time_index, position_matched_index);
        stitch_end_point += 1;
    }
    else
    {
        stitch_end_point = forward_time_index + 1;
    }

    std::vector<TrajectoryPoint> stitching_trajectory(
            prev_trajectory->begin() + stitch_start_point,
            prev_trajectory->begin() + stitch_end_point);

#if debug_traj_stitch
    AINFO << "forward_rel_time: " << forward_rel_time;
    AINFO << "forward_time_index: " << forward_time_index;

    AINFO << "veh speed: " << vehicle_state.linear_velocity();
    AINFO << "Position matched index: " << position_matched_index;
    AINFO << "Time matched index: " << time_matched_index;
    AINFO << "stitch_start_point: " << stitch_start_point;
    AINFO << "stitch_end_point: " << stitch_end_point;

    for (size_t i = 0; i < prev_trajectory->size(); i++)
    {
        AINFO << "id: " << i;
        AINFO << prev_trajectory->at(i).DebugString();
    }

    AINFO << "stitching_trajectory size: " << stitching_trajectory.size();

#endif
    // 轨迹点的时间基准是：当前帧时间戳.
    // 轨迹点的s基准是：历史轨迹上的车辆期望点

    const double zero_s = stitching_trajectory.back().path_point().s();
    for (auto& tp : stitching_trajectory)
    {
        if (!tp.has_path_point())
        {
            *replan_reason = "replan for previous trajectory missed path point";
            return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                                    vehicle_state);
        }
        tp.set_relative_time(tp.relative_time() +
                             prev_trajectory->header_time() -
                             current_timestamp);
        tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);

        // AINFO << tp.relative_time();
        // AINFO << tp.path_point().s();
    }
    return stitching_trajectory;
}

// 得到投影的sl，s相对于 plan start
std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
        const double x, const double y, const TrajectoryPoint& p)
{
    Vec2d v(x - p.path_point().x(), y - p.path_point().y());
    Vec2d n(std::cos(p.path_point().theta()), std::sin(p.path_point().theta()));

    std::pair<double, double> frenet_sd;
    frenet_sd.first = v.InnerProd(n) + p.path_point().s();
    frenet_sd.second = v.CrossProd(n);
    return frenet_sd;
}

}  // namespace planning
}  // namespace apollo
