/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file speed_profile_generator.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo
{
namespace planning
{



class SpeedProfileGenerator
{
public:
    // SpeedProfileGenerator() = delete;
    SpeedProfileGenerator() = default;

    static SpeedData GenerateFallbackSpeed(const EgoInfo* ego_info,
                                           const double stop_distance = 0.0);

    static void FillEnoughSpeedPoints(SpeedData* const speed_data);

    static SpeedData GenerateFixedDistanceCreepProfile(const double distance,
                                                       const double max_speed);

    double Evaluate(const std::uint32_t order, const double param);

    int init_speed_bound(double init_v);

    // 生成减速曲线
    SpeedData generate_constant_jerk_brake_profile(const double init_speed,
                                                   const double init_acc,
                                                   const double const_jerk);

    // speed up or speed down
    SpeedData generate_constant_acc_profile(const double init_speed,
                                            const double acc);

    // speed up or speed down
    // if speed up, expected_speed is speed limit
    // if speed down, expected_speed is min(obs_v, speed limit)
    const SpeedData speed_sampling_constant_acc(const double init_speed,
                                                 const double acc,
                                                 const double speed_limit,
                                                 const double sampling_time);

    // 2ms左右,比较耗时
    const SpeedData generate_constant_jerk_brake_profile_by_qp(
            const double init_speed, const double init_acc,
            const double const_jerk, const double path_length,
            const speed_mode speed_mode_);

    // 2ms左右,比较耗时
    const SpeedData generate_speed_up_profile_by_qp(const double init_speed,
                                                     const double init_acc,
                                                     const double path_length);

    bool check_speed_state_valid(const std::uint32_t order, double x);

    // 得到s 差分，v 差分，acc差分的值
    double get_lon_state_difference(const double start_state,
                                    const double end_state,
                                    const double delta_time);

    int set_lower_acc(double acc);

    int generate_zero_speed_profile(SpeedData *speed_profile);

private:
    static SpeedData GenerateStopProfile(const double init_speed,
                                         const double init_acc);

    double p0_;
    double v0_;
    double a0_;



    double param_;

    double jerk_;

    // 动力学限制，并不包含曲率限速、障碍物限速、路网限速
    speed_point_limit speed_point_limit_;


};

}  // namespace planning
}  // namespace apollo
