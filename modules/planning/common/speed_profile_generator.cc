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
 * @file speed_profile_generator.cc
 **/

#include "modules/planning/common/speed_profile_generator.h"

#include <algorithm>
#include <utility>

#include "cyber/common/log.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace apollo
{
namespace planning
{
using apollo::common::SpeedPoint;

#define debug_speed_profile_generator (0)
#define debug_speed_up_profile (0)

int SpeedProfileGenerator::generate_zero_speed_profile(SpeedData* speed_profile)
{
    if (speed_profile == nullptr)
    {
        return 0;
    }

    speed_profile->AppendSpeedPoint(0.0, 0.0, 0.0, 0.0, 0.0);
    FillEnoughSpeedPoints(speed_profile);

    return 0;
}

SpeedData SpeedProfileGenerator::GenerateFallbackSpeed(
        const EgoInfo* ego_info, const double stop_distance)
{
    AINFO << "Fallback using piecewise jerk speed!";

    const double init_v = ego_info->start_point().v();
    const double init_a = ego_info->start_point().a();

    AINFO << "init_v = " << init_v << ", init_a = " << init_a;
    
    const auto& veh_param =
            common::VehicleConfigHelper::GetConfig().vehicle_param();

    // if already stopped
    if (init_v <= 0.0 && init_a <= 0.0)
    {
        AINFO << "Already stopped! Nothing to do in GenerateFallbackSpeed()";
        SpeedData speed_data;
        speed_data.AppendSpeedPoint(0.0, 0.0, 0.0, 0.0, 0.0);
        FillEnoughSpeedPoints(&speed_data);
        return speed_data;
    }

    // init state:s, ds, dds
    std::array<double, 3> init_state = {0.0, init_v, init_a};

    // TODO(all): dt is too small;
    double delta_t = 0.1;
    double total_time = 7.0;
    const size_t num_of_knots = static_cast<size_t>(total_time / delta_t) + 1;

    PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,
                                                     init_state);

    double fallback_acc = 0.3;
    double s_upper = 0.5 * init_v * init_v / fallback_acc;

    std::vector<double> state_ref(num_of_knots, s_upper);
    piecewise_jerk_problem.set_x_ref(1.0, std::move(state_ref));

    piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

    double stop_distance_error = 1.0;

    piecewise_jerk_problem.set_x_bounds(0.0, s_upper);

    double speed_limit;
    speed_limit = std::fmax(FLAGS_planning_upper_speed_limit, init_v + 0.1);

    piecewise_jerk_problem.set_dx_bounds(0.0, speed_limit);

    piecewise_jerk_problem.set_ddx_bounds(veh_param.max_deceleration(),
                                          veh_param.max_acceleration());

    piecewise_jerk_problem.set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound,
                                          FLAGS_longitudinal_jerk_upper_bound);

    // Solve the problem
    if (!piecewise_jerk_problem.Optimize())
    {
        AERROR << "Piecewise jerk fallback speed optimizer failed!";

        return GenerateStopProfile(init_v, init_a);
    }

    // Extract output
    const std::vector<double>& s = piecewise_jerk_problem.opt_x();
    const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
    const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

#if debug_speed_profile_generator
    for (size_t i = 0; i < num_of_knots; ++i)
    {
        AINFO << "For[" << delta_t * static_cast<double>(i)
               << "], s = " << s[i] << ", v = " << ds[i] << ", a = " << dds[i];
    }
#endif

    SpeedData speed_data;
    speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
    for (size_t i = 1; i < num_of_knots; ++i)
    {
        // Avoid the very last points when already stopped
        if (s[i] - s[i - 1] <= 0.0 || ds[i] <= 0.0)
        {
            break;
        }
        speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i),
                                    ds[i], dds[i],
                                    (dds[i] - dds[i - 1]) / delta_t);
    }

    FillEnoughSpeedPoints(&speed_data);

    return speed_data;
}

void SpeedProfileGenerator::FillEnoughSpeedPoints(SpeedData* const speed_data)
{
    const SpeedPoint& last_point = speed_data->back();
    if (last_point.t() >= FLAGS_fallback_total_time)
    {
        return;
    }

    double init_t = last_point.t() + FLAGS_fallback_time_unit;
    double end_s = last_point.s();

    for (double t = init_t; t < FLAGS_fallback_total_time;
         t += FLAGS_fallback_time_unit)
    {
        speed_data->AppendSpeedPoint(end_s, t, 0.0, 0.0, 0.0);
    }
}

SpeedData SpeedProfileGenerator::GenerateStopProfile(const double init_speed,
                                                     const double init_acc)
{
    AERROR << "Slowing down the car within a constant deceleration with "
              "fallback stopping profile.";

    SpeedData speed_data;

    double k_regular_max_time = 7.0;

    double max_t = k_regular_max_time;

    const double unit_t = 0.1;

    double pre_s = 0.0;
    double pre_v = init_speed;
    double acc = FLAGS_slowdown_profile_deceleration;

    speed_data.AppendSpeedPoint(0.0, 0.0, init_speed, init_acc, 0.0);
    for (double t = unit_t; t < max_t; t += unit_t)
    {
        double s = 0.0;
        double v = 0.0;
        s = std::fmax(pre_s,
                      pre_s + 0.5 * (pre_v + (pre_v + unit_t * acc)) * unit_t);

        v = std::fmax(0.0, pre_v + unit_t * acc);

        speed_data.AppendSpeedPoint(s, t, v, acc, 0.0);

        pre_s = s;
        pre_v = v;

        if (v <= 0.0)
        {
            break;
        }
    }

    // 对于fallback speed，可能只填充三秒
    FillEnoughSpeedPoints(&speed_data);
    return speed_data;
}

SpeedData SpeedProfileGenerator::generate_constant_acc_profile(
        const double init_speed, const double acc)
{

    SpeedData speed_data;

    double k_regular_max_time = 7.0;

    double max_t;

    max_t = 7.0;

    const double unit_t = 0.1;

    double pre_s = 0.0;
    double pre_v = init_speed;

    speed_data.AppendSpeedPoint(0.0, 0.0, init_speed, acc, 0.0);

    for (double t = unit_t; t < max_t; t += unit_t)
    {
        double s = 0.0;
        double v = 0.0;
        s = std::fmax(pre_s,
                      pre_s + 0.5 * (pre_v + (pre_v + unit_t * acc)) * unit_t);

        v = std::fmax(0.0, pre_v + unit_t * acc);

        speed_data.AppendSpeedPoint(s, t, v, acc, 0.0);

        pre_s = s;
        pre_v = v;

        if (v <= 0.0)
        {
            break;
        }
    }



    const SpeedPoint& last_point = speed_data.back();
    if (last_point.t() >= max_t)
    {
        return speed_data;
    }

    double init_t = last_point.t() + unit_t;
    double end_s = last_point.s();

    for (double t = init_t; t < max_t; t += unit_t)
    {
        speed_data.AppendSpeedPoint(end_s, t, 0.0, 0.0, 0.0);
    }

    return speed_data;
}

const SpeedData SpeedProfileGenerator::speed_sampling_constant_acc(
        const double init_speed, const double acc, const double speed_limit,
        const double sampling_time)
{

    SpeedData speed_data;

    double k_regular_max_time = 7.0;

    double max_t;

    max_t = 7.0;

    const double unit_t = 0.1;

    double pre_s = 0.0;
    double pre_v = init_speed;

    speed_data.AppendSpeedPoint(0.0, 0.0, init_speed, acc, 0.0);

    // max v， 加速曲线
    if (acc > 0.0)
    {
        double max_v_time = (speed_limit - init_speed) / acc;
        double max_v_s =
                init_speed * max_v_time + 0.5 * acc * max_v_time * max_v_time;

        double s = 0.0;
        double v = 0.0;

        for (double t = unit_t; t < sampling_time; t += unit_t)
        {
            v = std::fmax(0.0, pre_v + unit_t * acc);

            s = std::fmax(
                    pre_s,
                    pre_s + 0.5 * (pre_v + (pre_v + unit_t * acc)) * unit_t);

            if (v > speed_limit)
            {
                s = max_v_s;

                v = speed_limit;
            }


            speed_data.AppendSpeedPoint(s, t, v, acc, 0.0);

            pre_s = s;
            pre_v = v;

            if (v >= speed_limit)
            {
                break;
            }
        }
    }
    // min v, 减速曲线
    else if (acc < 0.0)
    {
        double min_v_time = -init_speed / acc;

        double min_v_s = -init_speed * init_speed / acc / 2;

        double s = 0.0;
        double v = 0.0;

        for (double t = unit_t; t < sampling_time; t += unit_t)
        {

            s = std::fmax(
                    pre_s,
                    pre_s + 0.5 * (pre_v + (pre_v + unit_t * acc)) * unit_t);

            v = std::fmax(0.0, pre_v + unit_t * acc);

            if (t >= min_v_time)
            {
                s = min_v_s;
                v = 0.0;
            }

            speed_data.AppendSpeedPoint(s, t, v, acc, 0.0);

            pre_s = s;
            pre_v = v;

            if (v <= 0.0)
            {
                break;
            }
        }
    }

    // 生成匀速曲线
    const SpeedPoint& last_point = speed_data.back();
    if (last_point.t() >= max_t)
    {
        return speed_data;
    }

    double init_t = last_point.t() + unit_t;
    double end_s = last_point.s();
    double end_v = last_point.v();

    for (double t = init_t; t < max_t; t += unit_t)
    {
        end_s += end_v * unit_t;

        speed_data.AppendSpeedPoint(end_s, t, end_v, 0.0, 0.0);
    }

    return speed_data;
}

SpeedData SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(
        const double distance, const double max_speed)
{
    static constexpr double kConstDeceleration = -0.8;  // (~3sec to fully stop)
    static constexpr double kProceedingSpeed = 2.23;  // (5mph proceeding speed)
    const double proceeding_speed = std::fmin(max_speed, kProceedingSpeed);
    const double distance_to_start_deceleration =
            proceeding_speed * proceeding_speed / kConstDeceleration / 2;
    bool is_const_deceleration_mode = distance < distance_to_start_deceleration;

    double a = kConstDeceleration;
    double t = 0.0;
    double s = 0.0;
    double v = proceeding_speed;

    static constexpr double kDeltaT = 0.1;

    SpeedData speed_data;
    while (s < distance && v > 0)
    {
        if (is_const_deceleration_mode)
        {
            speed_data.AppendSpeedPoint(s, t, v, a, 0.0);
            t += kDeltaT;
            double v_new = std::max(0.0, v + a * t);
            s += kDeltaT * (v + v_new) / 2;
            v = v_new;
        }
        else
        {
            speed_data.AppendSpeedPoint(s, t, v, 0.0, 0.0);
            t += kDeltaT;
            s += kDeltaT * v;
            if (distance - s < distance_to_start_deceleration)
                is_const_deceleration_mode = true;
        }
    }

    return speed_data;
}

double SpeedProfileGenerator::get_lon_state_difference(const double start_state,
                                                       const double end_state,
                                                       const double delta_time)
{
    double diff = (end_state - start_state) / delta_time;
    return diff;
}

double SpeedProfileGenerator::Evaluate(const std::uint32_t order,
                                          const double param)
{
    switch (order)
    {
        case 0:
        {
            return p0_ + v0_ * param + 0.5 * a0_ * param * param +
                   jerk_ * param * param * param / 6.0;
        }
        case 1:
        {
            return v0_ + a0_ * param + 0.5 * jerk_ * param * param;
        }
        case 2:
        {
            return a0_ + jerk_ * param;
        }
        case 3:
        {
            return jerk_;
        }
        default:
            return 0.0;
    }

    return 0.0;
}

bool SpeedProfileGenerator::check_speed_state_valid(const std::uint32_t order,
                                                    double x)
{
    switch (order)
    {
        // s
        case 0:
        {
            if (x < 0.0)
            {
                return false;
            }
        }
        break;
        // v
        case 1:
        {
            if (x < speed_point_limit_.v_lower)
            {
                return false;
            }
            else if (x > speed_point_limit_.v_upper)
            {
                return false;
            }
        }
        break;
        // acc
        case 2:
        {
            if (x < speed_point_limit_.acc_lower)
            {
                return false;
            }
            else if (x > speed_point_limit_.acc_upper)
            {
                return false;
            }
        }
        break;
        // jerk
        case 3:
        {
            if (x < speed_point_limit_.jerk_lower)
            {
                return false;
            }
            else if (x > speed_point_limit_.jerk_upper)
            {
                return false;
            }
        }
        break;
        default:
            return true;
    }

    return true;
}

SpeedData SpeedProfileGenerator::generate_constant_jerk_brake_profile(
        const double init_speed, const double init_acc, const double const_jerk)
{
    // init
    double max_t = 7.0;
    double unit_t = 0.1;

    p0_ = 0.0;
    v0_ = init_speed;
    a0_ = init_acc;
    param_ = unit_t;
    // constant jerk 运动，注意，要考虑acc bound和speed bound
    jerk_ = const_jerk;

    // generate first speed
    SpeedData speed_data;

    bool state_valid;

    speed_data.AppendSpeedPoint(0.0, 0.0, init_speed, init_acc, 0.0);

    // generate next speed
    double p1_;
    double v1_;
    double a1_;

    for (double t = 0.1; t < max_t; t += unit_t)
    {
        double s = 0.0;
        double v = 0.0;

        a1_ = Evaluate(2, unit_t);

        // check acc
        state_valid = check_speed_state_valid(2, a1_);

        // if acc invalid
        if (!state_valid)
        {
            jerk_ = 0.0;
            a1_ = Evaluate(2, 0.1);
        }

        v1_ = Evaluate(1, unit_t);

        // check v
        state_valid = check_speed_state_valid(1, v1_);

        // if v invalid
        if (!state_valid)
        {
            jerk_ = 0.0;
            a0_ = 0.0;

            v1_ = Evaluate(1, 0.1);
        }

        p1_ = Evaluate(0, unit_t);

        // check s
        state_valid = check_speed_state_valid(0, p1_);

        // if s invalid
        if (!state_valid)
        {
            jerk_ = 0.0;
            a0_ = 0.0;
            v0_ = 0.0;

            p1_ = Evaluate(0, unit_t);
        }

        speed_data.AppendSpeedPoint(p1_, t, v1_, a1_, jerk_);

        p0_ = p1_;
        v0_ = v1_;
        a0_ = a1_;

        if (v1_ <= 0.0 || p1_ <= 0.0)
        {
            break;
        }
    }

#if debug_speed_profile_generator

    speed_data.log_speed_data();
#endif

    const SpeedPoint& last_point = speed_data.back();
    if (last_point.t() >= max_t)
    {
        return speed_data;
    }

    double init_t = last_point.t() + unit_t;
    double end_s = last_point.s();

    for (double t = init_t; t < max_t; t += unit_t)
    {
        speed_data.AppendSpeedPoint(end_s, t, 0.0, 0.0, 0.0);
    }

    return speed_data;
}

const SpeedData
SpeedProfileGenerator::generate_constant_jerk_brake_profile_by_qp(
        const double init_speed, const double init_acc, const double const_jerk,
        const double path_length, const speed_mode speed_mode_)
{
    // init
    double max_t = 7.0;
    double unit_t = 0.1;

    p0_ = 0.0;
    v0_ = init_speed;
    a0_ = init_acc;

    a0_ = std::min(speed_point_limit_.acc_upper, init_acc);
    a0_ = std::max(speed_point_limit_.acc_lower, init_acc);

    param_ = unit_t;
    // constant jerk 运动，注意，要考虑acc bound和speed bound
    jerk_ = const_jerk;


    // init state:s, ds, dds
    std::array<double, 3> init_state = {0.0, init_speed, a0_};

    // TODO(all): dt is too small;
    double delta_t = unit_t;
    double total_time = max_t;
    const size_t num_of_knots = static_cast<size_t>(total_time / delta_t) + 1;

    PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,
                                                     init_state);

    // end state并不一定能够满足，暂时不需要设置
    // std::array<double, 3> end_state = {0.0, init_speed, init_acc};
    // piecewise_jerk_problem.set_end_state_ref({10.0, 10.0, 0.0}, end_state);

    // ref s 设置为0
    std::vector<double> state_ref(num_of_knots, 0.0);
    // piecewise_jerk_problem.set_x_ref(100.0, std::move(state_ref));

    // ref v 设置为0
    piecewise_jerk_problem.set_dx_ref(1000.0, 0.0);

    piecewise_jerk_problem.set_x_bounds(0.0, path_length + 1.0);


    piecewise_jerk_problem.set_dx_bounds(0.0, speed_point_limit_.v_upper);

    double acc_lower = speed_point_limit_.acc_lower;

    switch (speed_mode_)
    {
    case speed_mode::hard_brake:
        acc_lower = speed_point_limit_.acc_lower;
        break;
    case speed_mode::soft_brake:
        acc_lower = -2.0;
        break;
    
    default:
        break;
    }

    piecewise_jerk_problem.set_ddx_bounds(acc_lower,
                                          speed_point_limit_.acc_upper);

    piecewise_jerk_problem.set_dddx_bound(speed_point_limit_.jerk_lower,
                                          speed_point_limit_.jerk_upper);

    // s越小越好
    piecewise_jerk_problem.set_weight_x(0.0);
    // v越小越好
    piecewise_jerk_problem.set_weight_dx(1.0);
    piecewise_jerk_problem.set_weight_ddx(10.0);
    piecewise_jerk_problem.set_weight_dddx(0.001);

    // Solve the problem
    if (!piecewise_jerk_problem.Optimize())
    {
        AERROR << "brake jerk fallback speed optimizer failed!";

        return generate_constant_jerk_brake_profile(init_speed, init_acc,
                                                    const_jerk);
    }

    // Extract output
    const std::vector<double>& s = piecewise_jerk_problem.opt_x();
    const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
    const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

    SpeedData speed_data;
    speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
    for (size_t i = 1; i < num_of_knots; ++i)
    {
        // Avoid the very last points when already stopped
        if (s[i] - s[i - 1] <= 0.0 || ds[i] <= 0.0)
        {
            break;
        }
        speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i),
                                    ds[i], dds[i],
                                    (dds[i] - dds[i - 1]) / delta_t);
    }

#if debug_speed_profile_generator

    speed_data.log_speed_data();
#endif

    const SpeedPoint& last_point = speed_data.back();
    if (last_point.t() >= max_t)
    {
        return speed_data;
    }

    double init_t = last_point.t() + unit_t;
    double end_s = last_point.s();

    for (double t = init_t; t < max_t; t += unit_t)
    {
        speed_data.AppendSpeedPoint(end_s, t, 0.0, 0.0, 0.0);
    }

    return speed_data;
}

const SpeedData SpeedProfileGenerator::generate_speed_up_profile_by_qp(
        const double init_speed, const double init_acc,
        const double path_length)
{
    // init
    double max_t = 7.0;
    double unit_t = 0.1;

    // init state:s, ds, dds
    std::array<double, 3> init_state = {0.0, init_speed, init_acc};

    // TODO(all): dt is too small;
    double delta_t = unit_t;
    double total_time = max_t;
    const size_t num_of_knots = static_cast<size_t>(total_time / delta_t) + 1;

    PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,
                                                     init_state);

    std::vector<double> state_ref(num_of_knots, path_length);
    piecewise_jerk_problem.set_x_ref(100.0, std::move(state_ref));

    piecewise_jerk_problem.set_x_bounds(0.0, path_length + 1.0);


    piecewise_jerk_problem.set_dx_bounds(0.0, speed_point_limit_.v_upper);

    piecewise_jerk_problem.set_ddx_bounds(speed_point_limit_.acc_lower,
                                          speed_point_limit_.acc_upper);

    piecewise_jerk_problem.set_dddx_bound(speed_point_limit_.jerk_lower,
                                          speed_point_limit_.jerk_upper);

    piecewise_jerk_problem.set_weight_x(0.0);
    piecewise_jerk_problem.set_weight_dx(0.0);
    piecewise_jerk_problem.set_weight_ddx(0.0);
    piecewise_jerk_problem.set_weight_dddx(0.0);

    // Solve the problem
    if (!piecewise_jerk_problem.Optimize())
    {
        AERROR << "speed optimizer failed!";

        return speed_sampling_constant_acc(init_speed,
                                           speed_point_limit_.acc_upper,
                                           speed_point_limit_.v_upper, 7.0);
    }

    // Extract output
    const std::vector<double>& s = piecewise_jerk_problem.opt_x();
    const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
    const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

    SpeedData speed_data;
    speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
    for (size_t i = 1; i < num_of_knots; ++i)
    {
        // Avoid the very last points when already stopped
        if (s[i] - s[i - 1] <= 0.0 || ds[i] <= 0.0)
        {
            break;
        }
        speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i),
                                    ds[i], dds[i],
                                    (dds[i] - dds[i - 1]) / delta_t);
    }


#if debug_speed_up_profile

    speed_data.log_speed_data();
#endif

    const SpeedPoint& last_point = speed_data.back();
    if (last_point.t() >= max_t)
    {
        return speed_data;
    }

    double init_t = last_point.t() + unit_t;
    double end_s = last_point.s();
    double end_v = last_point.v();

    for (double t = init_t; t < max_t; t += unit_t)
    {
        end_s += end_v * unit_t;

        speed_data.AppendSpeedPoint(end_s, t, end_v, 0.0, 0.0);
    }



    return speed_data;
}

int SpeedProfileGenerator::init_speed_bound(double init_v)
{
    speed_point_limit_.v_lower = 0.0;

    double speed_limit;
    speed_limit = std::fmax(FLAGS_planning_upper_speed_limit, init_v + 0.1);

    speed_point_limit_.v_upper = speed_limit;

    const auto& veh_param =
            common::VehicleConfigHelper::GetConfig().vehicle_param();

    speed_point_limit_.acc_upper = veh_param.max_acceleration();
    speed_point_limit_.acc_lower = veh_param.max_deceleration();

    speed_point_limit_.jerk_lower = FLAGS_longitudinal_jerk_lower_bound;

    speed_point_limit_.jerk_upper = FLAGS_longitudinal_jerk_upper_bound;

    return 0;
}

int SpeedProfileGenerator::set_lower_acc(double acc)
{


    speed_point_limit_.acc_lower = acc;


    return 0;
}

}  // namespace planning
}  // namespace apollo
