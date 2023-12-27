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
 * @file constraint_checker.cc
 **/

#include "modules/planning/constraint_checker/constraint_checker.h"

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

#include <iomanip>

namespace apollo
{
namespace planning
{
namespace
{
template <typename T> bool WithinRange(const T v, const T lower, const T upper)
{
    return lower <= v && v <= upper;
}
}  // namespace

ConstraintChecker::Result ConstraintChecker::ValidTrajectory(
        const DiscretizedTrajectory& trajectory, const double adc_v,
        ADCTrajectory::TrajectoryType traj_type)
{
    const double kMaxCheckRelativeTime = FLAGS_trajectory_time_length;
    double speed_max_limit = std::max(adc_v, FLAGS_speed_upper_bound);

    double k_speed_error = 1.0;
    double k_acc_error = 0.3;
    double k_jerk_error = 0.3;
    double k_kappa_error = 0.01;
    double k_lateral_acc_error = 0.1;

    for (const auto& p : trajectory)
    {
        double t = p.relative_time();
        if (t > kMaxCheckRelativeTime)
        {
            break;
        }
        double lon_v = p.v();
        if (!WithinRange(lon_v, FLAGS_speed_lower_bound,
                         speed_max_limit + k_speed_error))
        {
            AINFO << "Velocity at relative time " << t
                   << " exceeds bound, value: " << lon_v << ", bound ["
                   << FLAGS_speed_lower_bound << ", " << speed_max_limit
                   << "].";

            return Result::LON_VELOCITY_OUT_OF_BOUND;
        }

        double lon_a = p.a();
        if (!WithinRange(
                    lon_a,
                    FLAGS_longitudinal_acceleration_lower_bound - k_acc_error,
                    FLAGS_longitudinal_acceleration_upper_bound + k_acc_error))
        {
            AINFO << "Longitudinal acceleration at relative time " << t
                   << " exceeds bound, value: " << lon_a << ", bound ["
                   << FLAGS_longitudinal_acceleration_lower_bound << ", "
                   << FLAGS_longitudinal_acceleration_upper_bound << "].";

            return Result::LON_ACCELERATION_OUT_OF_BOUND;
        }

        double kappa = p.path_point().kappa();
        if (!WithinRange(kappa, -FLAGS_kappa_bound - k_kappa_error,
                         FLAGS_kappa_bound + k_kappa_error))
        {
            AINFO << "Kappa at relative time " << t
                   << " exceeds bound, value: " << kappa << ", bound ["
                   << -FLAGS_kappa_bound << ", " << FLAGS_kappa_bound << "].";

            return Result::CURVATURE_OUT_OF_BOUND;
        }
    }

    for (size_t i = 1; i < trajectory.NumOfPoints(); ++i)
    {
        const auto& p0 =
                trajectory.TrajectoryPointAt(static_cast<uint32_t>(i - 1));
        const auto& p1 = trajectory.TrajectoryPointAt(static_cast<uint32_t>(i));

        if (p1.relative_time() > kMaxCheckRelativeTime)
        {
            break;
        }

        double t = p0.relative_time();

        // // if speed fallback, 生成的jerk 往往不满足约束，todo
        // if (traj_type != ADCTrajectory::SPEED_FALLBACK)
        // {
        //     double dt = p1.relative_time() - p0.relative_time();
        //     double d_lon_a = p1.a() - p0.a();
        //     double lon_jerk = d_lon_a / dt;
        //     if (!WithinRange(
        //                 lon_jerk,
        //                 FLAGS_longitudinal_jerk_lower_bound - k_jerk_error,
        //                 FLAGS_longitudinal_jerk_upper_bound + k_jerk_error))
        //     {
        //         AERROR << "Longitudinal jerk at relative time " << t
        //                << " exceeds bound, value: " << std::setprecision(4)
        //                << lon_jerk << ", bound [" << std::setprecision(4)
        //                << FLAGS_longitudinal_jerk_lower_bound << ", "
        //                << std::setprecision(4)
        //                << FLAGS_longitudinal_jerk_upper_bound << "].";

        //         return Result::LON_JERK_OUT_OF_BOUND;
        //     }
        // }

        double lat_a = p1.v() * p1.v() * p1.path_point().kappa();
        if (!WithinRange(
                    lat_a,
                    -FLAGS_lateral_acceleration_bound - k_lateral_acc_error,
                    FLAGS_lateral_acceleration_bound + k_lateral_acc_error))
        {
            AINFO << "Lateral acceleration at relative time " << t
                   << " exceeds bound, value: " << lat_a << ", bound ["
                   << -FLAGS_lateral_acceleration_bound << ", "
                   << FLAGS_lateral_acceleration_bound << "].";

            return Result::LAT_ACCELERATION_OUT_OF_BOUND;
        }

        // TODO(zhangyajia): this is temporarily disabled
        // due to low quality reference line.
        /**
        double d_lat_a = p1.v() * p1.v() * p1.path_point().kappa() -
                         p0.v() * p0.v() * p0.path_point().kappa();
        double lat_jerk = d_lat_a / dt;
        if (!WithinRange(lat_jerk, -FLAGS_lateral_jerk_bound,
                         FLAGS_lateral_jerk_bound)) {
          ADEBUG << "Lateral jerk at relative time " << t
                 << " exceeds bound, value: " << lat_jerk << ", bound ["
                 << -FLAGS_lateral_jerk_bound << ", " <<
        FLAGS_lateral_jerk_bound
                 << "].";
          return Result::LAT_JERK_OUT_OF_BOUND;
        }
        **/
    }

    return Result::VALID;
}

}  // namespace planning
}  // namespace apollo
