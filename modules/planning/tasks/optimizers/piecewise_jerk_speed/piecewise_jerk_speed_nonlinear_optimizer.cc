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

/**
 * @file piecewise_jerk_fallback_speed.cc
 **/

#include "modules/planning/tasks/optimizers/piecewise_jerk_speed/piecewise_jerk_speed_nonlinear_optimizer.h"

#include <algorithm>
#include <string>

#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_path_problem.h"
#include "modules/planning/proto/ipopt_return_status.pb.h"
#include "modules/planning/tasks/optimizers/piecewise_jerk_speed/piecewise_jerk_speed_nonlinear_ipopt_interface.h"

#include "modules/planning/lattice/trajectory_generation/lattice_trajectory1d.h"

namespace apollo
{
namespace planning
{
using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

#define debug_pwj_speed_nlp (0)
#define debug_pwj_speed_qp (0)

#define debug_speed_data (0)
#define debug_nlp_speed_constraint (0)

#define debug_lattice (0)

#define enable_speed_nlp (1)

#define enable_nlp_speed_opt_by_qp (1)
#define enable_speed_decision (1)

constexpr double k_overtake_safe_dist = 10.0;

PiecewiseJerkSpeedNonlinearOptimizer::PiecewiseJerkSpeedNonlinearOptimizer(
        const TaskConfig& config) :
    SpeedOptimizer(config),
    smoothed_speed_limit_(0.0, 0.0, 0.0),
    smoothed_path_curvature_(0.0, 0.0, 0.0)
{
    ACHECK(config_.has_piecewise_jerk_nonlinear_speed_optimizer_config());
}

bool PiecewiseJerkSpeedNonlinearOptimizer::check_constant_speed_profile_is_nice(
        const SpeedData& speed_data, const speed_lattice_cost& speed_cost)
{
    if (speed_cost.min_lon_gap < 2.0)
    {
        return false;
    }

    if (speed_cost.end_point_time_headway >= 3.0)
    {
        return true;
    }

    return true;
}

bool PiecewiseJerkSpeedNonlinearOptimizer::check_speed_down_profile_is_nice(
        const SpeedData& speed_data, const speed_lattice_cost& speed_cost)
{
    // collision
    if (speed_cost.min_lon_gap < 0.3)
    {
        return false;
    }

    if (speed_cost.end_point_time_headway >= 3.0)
    {
        return true;
    }

    return true;
}

bool PiecewiseJerkSpeedNonlinearOptimizer::check_speed_up_profile_is_nice(
        const SpeedData& speed_data, const speed_lattice_cost& speed_cost)
{
    if (speed_cost.min_lon_gap < 2.0)
    {
        return false;
    }

    if (speed_cost.min_time_headway < 2.0)
    {
        return false;
    }

    if (speed_cost.start_point_time_headway >= 2.0 &&
        speed_cost.end_point_time_headway < 2.0)
    {
        return false;
    }

    if (speed_cost.end_point_time_headway <= 3.0)
    {
        return true;
    }

    return true;
}

double PiecewiseJerkSpeedNonlinearOptimizer::estimate_speed_limit_cost(
        const SpeedData& adc_speed_data)
{
    double speed_limit_total_cost = 0.0;

    double s;

    double point_speed_limit;

    double weight = 1.0;

    double cost;
    double delta_v;

    for (size_t i = 0; i < adc_speed_data.size(); i++)
    {
        s = adc_speed_data[i].s();

        point_speed_limit = speed_limit_.GetSpeedLimitByS(s);

        if (adc_speed_data[i].v() > point_speed_limit)
        {
            delta_v = adc_speed_data[i].v() - point_speed_limit;
            cost = weight * delta_v * delta_v;

            speed_limit_total_cost += cost;
        }
    }

    return speed_limit_total_cost;
}

int PiecewiseJerkSpeedNonlinearOptimizer::
        estimate_min_lon_gap_for_follow_decision(
                speed_lattice_cost* speed_cost, const Obstacle& obstacle,
                const SpeedData& adc_speed_data)
{
    double time;

    // init
    time = -0.1;

    const STBoundary& st_bound = obstacle.path_st_boundary();

    double obs_s_upper;
    double obs_s_lower;

    double ego_s;

    bool has_interaction;
    bool update_first_point = false;

    double thd = 100.0;

    double cur_gap;
    double min_gap = 1000.0;

    // init
    speed_cost->start_point_lon_gap = 1000.0;
    speed_cost->end_point_lon_gap = 1000.0;
    speed_cost->min_lon_gap = 1000.0;

    size_t adc_speed_point_size = adc_speed_data.size();

    for (size_t i = 0; i < adc_speed_point_size; i++)
    {
        time = adc_speed_data[i].t();

        has_interaction =
                st_bound.GetBoundarySRange(time, &obs_s_upper, &obs_s_lower);

        if (!has_interaction)
        {
            continue;
        }

        ego_s = adc_speed_data[i].s();

        cur_gap = obs_s_lower - ego_s;

        if (adc_speed_data[i].v() > 0.1)
        {
            thd = cur_gap / adc_speed_data[i].v();
        }
        else
        {
            thd = 1000.0;
        }

        // start point
        if (!update_first_point)
        {
            speed_cost->start_point_lon_gap = cur_gap;

            speed_cost->start_point_time_headway = thd;

            update_first_point = true;
        }

        // end point
        {
            speed_cost->end_point_lon_gap = cur_gap;

            speed_cost->end_point_time_headway = thd;
        }

        if (cur_gap < min_gap)
        {
            min_gap = cur_gap;

            speed_cost->min_lon_gap = min_gap;
            speed_cost->min_time_headway = thd;
        }
    }

    return 0;
}

int PiecewiseJerkSpeedNonlinearOptimizer::speed_lattice_process(
        SpeedData* const speed_data, const double speed_limit)
{
    SpeedProfileGenerator speed_generate;
    SpeedData sampling_speed;
    SpeedData best_speed;
    SpeedData constant_speed_profile;

    // 加速采样
    std::vector<double> sampling_acc_list = {0.1, 0.2, 0.3, 0.4, 0.5};

    // 减速采样
    std::vector<double> sampling_dec_list = {-0.1, -0.2, -0.3, -0.5, -0.8, -1.2,
                                             -1.5, -1.8, -2.1, -2.4, -3.0, -3.5,
                                             -4.0, -4.5, -5.0, -5.5, -6.0};

    // 采样时间
    std::vector<double> sampling_time_list = {1.0, 2.0, 3.0, 4.0,
                                              5.0, 6.0, 7.0};

    const auto time_start = std::chrono::system_clock::now();

    double min_lon_gap;

    const Obstacle* following_obs =
            reference_line_info_->get_mutable_close_following_obstacle();

    // no following scenario
    if (following_obs == nullptr)
    {
        return 0;
    }

    double obs_v = following_obs->speed();

    double obs_s;
    obs_s = following_obs->path_st_boundary().min_s();

    interaction_secenerio_type following_mode =
            INTERACTION_SECENERIO_SPEED_KEEP_FOLLOW;

    // update speed mode

    if (following_obs->get_time_of_headway() < 2.5 ||
        following_obs->get_time_to_collision() < 3.0)
    {
        following_mode = INTERACTION_SECENERIO_SPEED_DOWN_FOLLOW;
    }
    else if (following_obs->get_time_of_headway() > 3.0 &&
             s_dot_init_ < speed_limit)
    {
        following_mode = INTERACTION_SECENERIO_SPEED_UP_FOLLOW;
    }
    else
    {
        following_mode = INTERACTION_SECENERIO_SPEED_KEEP_FOLLOW;
    }

    // generate constant speed profile

    constant_speed_profile = speed_generate.speed_sampling_constant_acc(
            s_dot_init_, 0.0, max_speed_, 7.0);

    // cost
    speed_lattice_cost constant_speed_profile_cost;

    estimate_min_lon_gap_for_follow_decision(&constant_speed_profile_cost,
                                             *following_obs,
                                             constant_speed_profile);

    constant_speed_profile_cost.sampling_acc = 0.0;

    // check speed limit
    constant_speed_profile_cost.speed_limit =
            estimate_speed_limit_cost(constant_speed_profile);


#if debug_lattice
    debug_speed_lattice_profile(constant_speed_profile_cost);

    following_obs->PerceptionSLBoundary().DebugString();

#endif


    // // if constant speed profile is nice, return
    // if (check_constant_speed_profile_is_nice(constant_speed_profile,
    //                                 constant_speed_profile_cost))
    // {
    //     *speed_data = constant_speed_profile;

    //     return 0;
    // }

    // check cut in

    const SLBoundary& sl_box = following_obs->PerceptionSLBoundary();

    double lane_left_width;
    double lane_right_width;
    bool is_cut_in_obs = false;

    if (!reference_line_info_->is_path_lane_borrow())
    {
        if (reference_line_info_->reference_line().GetLaneWidth(
                    sl_box.start_s(), &lane_left_width, &lane_right_width))
        {
            double middle_l = (sl_box.start_l() + sl_box.end_l()) / 2;
            if (sl_box.end_l() > lane_left_width && middle_l > 1.0)
            {
                is_cut_in_obs = true;
            }
            if (sl_box.start_l() < -lane_right_width && middle_l < -1.0)
            {
                is_cut_in_obs = true;
            }
        }
    }
    else
    {
        if (sl_box.start_l() > reference_line_info_->AdcSlBoundary().end_l() ||
            sl_box.end_l() < reference_line_info_->AdcSlBoundary().start_l())
        {
            is_cut_in_obs = true;
        }
        else
        {
            is_cut_in_obs = false;
        }
    }

    // generate constant acceleration speed profile

    double acc;

    speed_lattice_cost speed_profile_cost;
    speed_lattice_cost speed_profile_best_cost;

    double time;

    bool has_nice_speed = false;

    switch (following_mode)
    {
        case INTERACTION_SECENERIO_SPEED_KEEP_FOLLOW:
            best_speed = constant_speed_profile;

            break;
        case INTERACTION_SECENERIO_SPEED_UP_FOLLOW:

            speed_profile_best_cost = constant_speed_profile_cost;
            best_speed = constant_speed_profile;

            if (constant_speed_profile_cost.end_point_time_headway <= 3.0)
            {
#if debug_lattice
                debug_speed_lattice_profile(constant_speed_profile_cost);

#endif
                break;
            }

            for (size_t j = 0; j < sampling_acc_list.size(); j++)
            {
                acc = sampling_acc_list[j];

                for (size_t i = 0; i < sampling_time_list.size(); i++)
                {
                    time = sampling_time_list[i];

                    sampling_speed = speed_generate.speed_sampling_constant_acc(
                            s_dot_init_, acc, max_speed_, time);

                    // check speed profile lon gap
                    estimate_min_lon_gap_for_follow_decision(
                            &speed_profile_cost, *following_obs,
                            sampling_speed);

                    // gap is small, will collision, so give up this profile
                    if (speed_profile_cost.min_lon_gap < 2.0)
                    {
                        continue;
                    }

                    speed_profile_cost.sampling_acc = acc;
                    speed_profile_cost.sampling_time = time;

                    // check speed limit
                    speed_profile_cost.speed_limit =
                            estimate_speed_limit_cost(sampling_speed);

#if debug_lattice
                    debug_speed_lattice_profile(speed_profile_cost);

#endif

                    if (!check_speed_up_profile_is_nice(sampling_speed,
                                                        speed_profile_cost))
                    {
                        continue;
                    }

                    double diff = std::fabs(
                            speed_profile_cost.end_point_time_headway - 3.0);

                    double best_diff = std::fabs(
                            speed_profile_best_cost.end_point_time_headway -
                            3.0);

                    if (diff < best_diff)
                    {
                        speed_profile_best_cost = speed_profile_cost;

                        best_speed = sampling_speed;
                    }
                }
            }

            break;
        case INTERACTION_SECENERIO_SPEED_DOWN_FOLLOW:

            // init
            speed_profile_best_cost = constant_speed_profile_cost;
            best_speed = constant_speed_profile;

            for (size_t j = 0; j < sampling_dec_list.size(); j++)
            {
                acc = sampling_dec_list[j];

                for (size_t i = 0; i < sampling_time_list.size(); i++)
                {
                    time = sampling_time_list[i];

                    sampling_speed = speed_generate.speed_sampling_constant_acc(
                            s_dot_init_, acc, max_speed_, time);

                    // check speed profile lon gap
                    estimate_min_lon_gap_for_follow_decision(
                            &speed_profile_cost, *following_obs,
                            sampling_speed);

                    speed_profile_cost.sampling_acc = acc;
                    speed_profile_cost.sampling_time = time;

#if debug_lattice
                    debug_speed_lattice_profile(speed_profile_cost);

#endif

                    // 总是出现速度error perception，result in hard brake
                    if (is_cut_in_obs)
                    {
                        if ((acc + 0.3) < 0.01 && std::fabs(time - 2.0) < 0.1)
                        {
                            best_speed = sampling_speed;

                            has_nice_speed = true;
                            break;
                        }
                    }

                    // if (acc == -0.3 && std::fabs(time - 7.0) < 0.1)
                    // {
                    //     sampling_speed.log_speed_data();
                    // }

                    // gap is small, will collision, so give up this profile
                    if (speed_profile_cost.min_lon_gap <
                                speed_profile_cost.start_point_lon_gap &&
                        speed_profile_cost.min_lon_gap < 0.3)
                    {
#if debug_lattice
                        AINFO << "min gap is small ";

#endif
                        continue;
                    }

                    // check speed limit
                    speed_profile_cost.speed_limit =
                            estimate_speed_limit_cost(sampling_speed);

                    if (!check_speed_down_profile_is_nice(sampling_speed,
                                                          speed_profile_cost))
                    {
#if debug_lattice
                        AINFO << "collision";

#endif
                        continue;
                    }



                    // 没有碰撞，且逐渐拉开距离，return
                    if (speed_profile_cost.end_point_time_headway >
                        speed_profile_cost.start_point_time_headway)
                    {
                        if (speed_profile_cost.end_point_time_headway <= 3.0 &&
                            speed_profile_cost.end_point_time_headway >= 2.0)
                        {
                            best_speed = sampling_speed;

                            has_nice_speed = true;
                            break;
                        }
                    }

                    // 末端速度满足time headway约束，return
                    if (speed_profile_cost.min_time_headway > 2.0)
                    {
                        if (speed_profile_cost.end_point_time_headway <= 3.0 &&
                            speed_profile_cost.end_point_time_headway >= 2.0)
                        {
                            best_speed = sampling_speed;

                            has_nice_speed = true;
                            break;
                        }
                    }

                    // 时距在增加，就行了，主要是cut in场景，不能急刹
                    if (speed_profile_cost.start_point_time_headway < 0.5)
                    {
                        if (speed_profile_cost.end_point_time_headway >
                            (speed_profile_cost.start_point_time_headway + 0.5))
                        {
                            best_speed = sampling_speed;

                            has_nice_speed = true;
                            break;
                        }
                    }

                    double diff = std::fabs(
                            speed_profile_cost.end_point_time_headway - 3.0);

                    speed_profile_cost.time_headway_cost = diff;

                    double best_diff = std::fabs(
                            speed_profile_best_cost.end_point_time_headway -
                            3.0);

                    speed_profile_best_cost.time_headway_cost = best_diff;

                    double acc_weight = 2.5;
                    double acc_cost =
                            -acc_weight * speed_profile_cost.sampling_acc;

                    speed_profile_cost.sampling_acc_cost = acc_cost;

                    speed_profile_best_cost.sampling_acc_cost =
                            -acc_weight * speed_profile_best_cost.sampling_acc;

                    if (speed_profile_cost.time_headway_cost +
                                speed_profile_cost.sampling_acc_cost <
                        speed_profile_best_cost.time_headway_cost +
                                speed_profile_best_cost.sampling_acc_cost)
                    {
                        speed_profile_best_cost = speed_profile_cost;

                        best_speed = sampling_speed;
                    }
                }

                if (has_nice_speed)
                {
                    break;
                }
            }
            break;

        default:
            break;
    }

    const auto time_end = std::chrono::system_clock::now();
    std::chrono::duration<double> time_diff = time_end - time_start;

    AINFO << "print_speed_sampling_time:"
          << "(" << time_diff.count() * 1000 << ","
          << ")";

    *speed_data = best_speed;

    return 0;
}

int PiecewiseJerkSpeedNonlinearOptimizer::post_process(
        SpeedData* const speed_data)
{
    // 特殊处理，ego和obs相距一百米不要加速
    // 因为obs速度在变,距离在变,ego速度也在变,这样会带来ego一直在加速,减速
    // 一直特殊处理是，osbtacle速度在变，距离在变，我们不要管它。具体策略是，如果相距100以内，
    // 生成的速度曲线是加速，说明ego即将会缩小跟车距离，这是不符合期望的，
    // ego保持匀速行驶即可
    if (reference_line_info_->get_speed_decision().has_type() &&
        reference_line_info_->get_speed_decision().type() ==
                INTERACTION_SECENERIO_FOLLOW)
    {
        // 100m内的处理

        const Obstacle* obs =
                reference_line_info_->get_mutable_close_following_obstacle();

        if (obs != nullptr)
        {
            AINFO << "obs id: " << obs->Id() << " ,ttc "
                  << obs->get_time_to_collision()
                  << ", time headwawy: " << obs->get_time_of_headway();
        }

        double obs_s;
        obs_s = 200.0;
        if (obs != nullptr)
        {
            obs_s = obs->path_st_boundary().min_s();
        }

#if enable_speed_decision

        double time_head_way = s_dot_init_ * 3.0;

        if (obs_s < time_head_way)
        {
            bool is_speed_up_profile = false;

            double acc = 0.0;

            SpeedPoint sp;

            speed_data->EvaluateByTime(0.1, &sp);

            acc += sp.a();

            speed_data->EvaluateByTime(0.3, &sp);

            acc += sp.a();
            speed_data->EvaluateByTime(1.0, &sp);

            acc += sp.a();

            if (acc > 0.1)
            {
                is_speed_up_profile = true;
                reference_line_info_->speed_decision_.set_type(
                        INTERACTION_SECENERIO_SPEED_KEEP_FOLLOW);
                reference_line_info_->speed_decision_.set_acc(0.0);
            }
            else if (acc < -0.1)
            {
                reference_line_info_->speed_decision_.set_type(
                        INTERACTION_SECENERIO_SPEED_DOWN_FOLLOW);

                reference_line_info_->speed_decision_.set_acc(-0.5);
            }
        }
        else
        {
            SpeedPoint sp;

            double acc = 0.0;

            speed_data->EvaluateByTime(0.1, &sp);

            acc += sp.a();

            speed_data->EvaluateByTime(0.3, &sp);

            acc += sp.a();
            speed_data->EvaluateByTime(0.5, &sp);

            acc += sp.a();

            if (acc > 0.1)
            {
                reference_line_info_->speed_decision_.set_type(
                        INTERACTION_SECENERIO_SPEED_UP_FOLLOW);
            }
            else if (acc < -0.1)
            {
                reference_line_info_->speed_decision_.set_type(
                        INTERACTION_SECENERIO_SPEED_DOWN_FOLLOW);

                reference_line_info_->speed_decision_.set_acc(-0.5);
            }

            double start_point_v;
            double diff_v;
            bool constant_v = true;

            size_t point_size = speed_data->size();

            for (size_t i = 0; i < point_size; i++)
            {
                const SpeedPoint& point = speed_data->at(i);

                if (i == 0)
                {
                    start_point_v = point.v();
                }
                else
                {
                    diff_v = point.v() - start_point_v;
                    if (diff_v > 0.1)
                    {
                        constant_v = false;
                        break;
                    }
                }

                if (point.s() > 10.0)
                {
                    break;
                }
            }

            if (constant_v)
            {
                reference_line_info_->speed_decision_.set_type(
                        INTERACTION_SECENERIO_SPEED_KEEP_FOLLOW);

                reference_line_info_->speed_decision_.set_acc(0.0);
            }
        }

#endif
    }

    return 0;
}

Status PiecewiseJerkSpeedNonlinearOptimizer::Process(
        const PathData& path_data, const TrajectoryPoint& init_point,
        SpeedData* const speed_data)
{
    if (speed_data == nullptr)
    {
        const std::string msg = "Null speed_data pointer";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    if (path_data.discretized_path().empty())
    {
        const std::string msg = "Speed Optimizer receives empty path data";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    if (reference_line_info_->ReachedDestination())
    {
        return Status::OK();
    }

    // speed bound
    const auto problem_setups_status =
            SetUpStatesAndBounds(path_data, *speed_data);

    record_constraints();

    if (!problem_setups_status.ok())
    {
        speed_data->clear();
        return problem_setups_status;
    }

    double speed_limit = max_speed_;

    const std::vector<std::pair<double, double>>& speed_limit_curve =
            speed_limit_.speed_limit_points();

    for (size_t i = 0; i < speed_limit_curve.size(); i++)
    {
        speed_limit = std::min(speed_limit, speed_limit_curve[i].second);
    }

    std::vector<double> distance;
    std::vector<double> velocity;
    std::vector<double> acceleration;

    std::array<double, 3> init_states = {s_init_, s_dot_init_, s_ddot_init_};

    Status qp_smooth_status;

    // 初值
    PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots_, delta_t_,
                                                     init_states);

    // following scenario, 没有需要stop/yield/overtake的决策，use lattice
    // planner
    if (reference_line_info_->get_speed_decision().has_type() &&
        reference_line_info_->get_speed_decision().type() ==
                INTERACTION_SECENERIO_FOLLOW &&
        s_dot_init_ < speed_limit && FLAGS_enable_speed_lattice_search)
    {
        speed_lattice_process(speed_data, speed_limit);

        // if acc is big, use qp smooth
        if (speed_data->size() > 0 && speed_data->begin()->a() < -0.5)
        {
            qp_smooth_status =
                    smooth_lattice_by_qp(speed_data, &distance, &velocity,
                                         &acceleration, piecewise_jerk_problem);

            if (qp_smooth_status.ok())
            {
                speed_data->clear();
                speed_data->AppendSpeedPoint(distance[0], 0.0, velocity[0],
                                             acceleration[0], 0.0);

                for (int i = 1; i < num_of_knots_; ++i)
                {
                    // Avoid the very last points when already stopped
                    if (velocity[i] < 0.0)
                    {
                        break;
                    }
                    speed_data->AppendSpeedPoint(
                            distance[i], delta_t_ * i, velocity[i],
                            acceleration[i],
                            (acceleration[i] - acceleration[i - 1]) / delta_t_);
                }
                SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
            }
        }

        return Status::OK();
    }

    // qp for dp result
    const auto qp_start = std::chrono::system_clock::now();

    qp_smooth_status = OptimizeByQP(speed_data, &distance, &velocity,
                                    &acceleration, piecewise_jerk_problem);

    // publish qp info to cyber rt
    record_qp_info(distance, velocity, acceleration);

    const auto qp_end = std::chrono::system_clock::now();
    std::chrono::duration<double> qp_diff = qp_end - qp_start;

    AINFO << "print_speed_qp_optimization:"
          << "(" << qp_diff.count() * 1000 << ","
          << ")";

    if (!qp_smooth_status.ok())
    {
        speed_data->clear();

        // add qp info to planning.log file
#if debug_pwj_speed_nlp
        debug_qp();

#endif

        return qp_smooth_status;
    }

#if enable_speed_nlp
    optimize_speed_by_nlp_interface(path_data, speed_data, &distance, &velocity,
                                    &acceleration);
#endif

    speed_data->clear();
    speed_data->AppendSpeedPoint(distance[0], 0.0, velocity[0], acceleration[0],
                                 0.0);

    for (int i = 1; i < num_of_knots_; ++i)
    {
        // Avoid the very last points when already stopped
        if (velocity[i] < 0.0)
        {
            break;
        }
        speed_data->AppendSpeedPoint(
                distance[i], delta_t_ * i, velocity[i], acceleration[i],
                (acceleration[i] - acceleration[i - 1]) / delta_t_);
    }
    SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);

#if enable_nlp_speed_opt_by_qp

    // check speed down curve
    SpeedPoint sp;

    double acc = 0.0;

    speed_data->EvaluateByTime(0.1, &sp);

    acc += sp.a();

    speed_data->EvaluateByTime(0.3, &sp);

    acc += sp.a();
    speed_data->EvaluateByTime(0.5, &sp);

    acc += sp.a();

    // 如果是减速，需要qp平滑
    if (acc < -0.1)
    {
        std::array<double, 3> init_states = {s_init_, s_dot_init_,
                                             s_ddot_init_};

        piecewise_jerk_problem.set_init_state(init_states);

        // ref line
        std::vector<double> x_ref;
        for (int i = 0; i < num_of_knots_; ++i)
        {
            const double curr_t = i * delta_t_;
            // get path_s
            SpeedPoint sp;
            speed_data->EvaluateByTime(curr_t, &sp);
            x_ref.emplace_back(sp.s());
        }

        // TODO(Jinyun): parameter tunnings
        const auto& config =
                config_.piecewise_jerk_nonlinear_speed_optimizer_config();

        piecewise_jerk_problem.set_x_ref(config.ref_s_weight(),
                                         std::move(x_ref));

        // UPDATE acc
        std::vector<double> sampling_acc_list = {-0.3, -0.5, -0.8, -1.2, -1.5,
                                                 -1.8, -2.1, -2.4, -3.0, -3.5,
                                                 -4.0, -4.5, -5.0, -5.5, -6.0};

        double sampling_dec;
        for (size_t i = 0; i < sampling_acc_list.size(); i++)
        {
            sampling_dec = sampling_acc_list[i];

            if (s_ddot_init_ < sampling_dec)
            {
                continue;
            }

            piecewise_jerk_problem.set_ddx_bounds(sampling_dec, s_ddot_max_);

            // Solve the problem
            if (!piecewise_jerk_problem.Optimize())
            {
                AINFO << " qp smooth speed fail by dec " << sampling_dec;
                continue;
            }
            else
            {
                AINFO << " qp smooth speed success by dec " << sampling_dec;

                distance = piecewise_jerk_problem.opt_x();
                velocity = piecewise_jerk_problem.opt_dx();
                acceleration = piecewise_jerk_problem.opt_ddx();

                speed_data->clear();
                speed_data->AppendSpeedPoint(distance[0], 0.0, velocity[0],
                                             acceleration[0], 0.0);

                for (int i = 1; i < num_of_knots_; ++i)
                {
                    // Avoid the very last points when already stopped
                    if (velocity[i] < 0.0)
                    {
                        break;
                    }
                    speed_data->AppendSpeedPoint(
                            distance[i], delta_t_ * i, velocity[i],
                            acceleration[i],
                            (acceleration[i] - acceleration[i - 1]) / delta_t_);
                }
                SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);

                // smooth_speed_data.log_speed_data();

                break;
            }
        }
    }

#endif

    post_process(speed_data);

    // StGraphData* st_graph_data =
    // reference_line_info_->mutable_st_graph_data();
    // RecordDebugInfo(*speed_data, st_graph_data->mutable_st_graph_debug());

#if debug_speed_data
    speed_data->log_speed_data();

#endif

    return Status::OK();
}

Status PiecewiseJerkSpeedNonlinearOptimizer::SetUpStatesAndBounds(
        const PathData& path_data, const SpeedData& speed_data)
{
    // Set st problem dimensions
    const StGraphData& st_graph_data =
            *reference_line_info_->mutable_st_graph_data();

    const SpeedData& emergency_brake_curve =
            reference_line_info_->emergency_brake_speed_data();

    double k_s_bound_error = 0.1;

    max_speed_ = reference_line_info_->get_max_speed();

    // TODO(Jinyun): move to confs
    delta_t_ = 0.1;
    total_length_ = st_graph_data.path_length();
    total_time_ = st_graph_data.total_time_by_conf();
    // 71个点:
    num_of_knots_ = static_cast<int>(total_time_ / delta_t_) + 1;

    // Set initial values
    s_init_ = 0.0;
    s_dot_init_ = st_graph_data.init_point().v();
    s_ddot_init_ = st_graph_data.init_point().a();

    // Set s_dot bounary, speed bound
    // 如果车辆超出限速，那么speed bound需要考虑车速，且要acc constraints, jerk
    // constraints.
    s_dot_max_ = std::max(max_speed_, s_dot_init_ + 1.0);

    // Set s_ddot boundary, acc bound
    const auto& veh_param =
            common::VehicleConfigHelper::GetConfig().vehicle_param();
    s_ddot_max_ = veh_param.max_acceleration();
    // use dec?
    s_ddot_min_ = -1.0 * std::abs(veh_param.max_deceleration());

    // Set s_dddot boundary, jerk bound
    // TODO(Jinyun): allow the setting of jerk_lower_bound and move jerk config
    // to a better place
    s_dddot_min_ = -std::abs(FLAGS_longitudinal_jerk_lower_bound);
    s_dddot_max_ = FLAGS_longitudinal_jerk_upper_bound;

    // Set s boundary，生成可行驶区域
    // default is 1
    if (FLAGS_use_soft_bound_in_nonlinear_speed_opt)
    {
        s_bounds_.clear();
        s_soft_bounds_.clear();

        double drive_s_lower = 0.0;
        double drive_s_upper = 0.0;

        double soft_follow_dist;

        bool has_dynamic_constraints_lower = false;

        SpeedPoint dynamic_constraints_lower_point;

        SpeedPoint dp_speed_point;

        bool is_interaction_end_point_time = false;

        st_gap_poi end_interaction_poi;

        // TODO(Jinyun): move to confs
        for (int i = 0; i < num_of_knots_; ++i)
        {
            double curr_t = i * delta_t_;
            double s_lower_bound = 0.0;
            double s_upper_bound = total_length_;
            double s_soft_lower_bound = 0.0;
            double s_soft_upper_bound = total_length_;

            if (!st_graph_data.is_st_boundaries_empty())
            {
                has_dynamic_constraints_lower =
                        emergency_brake_curve.EvaluateByTime(
                                curr_t, &dynamic_constraints_lower_point);

                if (!speed_data.EvaluateByTime(curr_t, &dp_speed_point))
                {
                    const std::string msg =
                            "rough speed profile estimation for soft "
                            "follow fence failed";
                    AERROR << msg;

                    return Status(ErrorCode::PLANNING_ERROR, msg);
                }
            }

            double follow_upper;
            double yield_upper;

            double s_gap;

            for (const STBoundary* boundary : st_graph_data.st_boundaries())
            {
                drive_s_lower = 0.0;
                drive_s_upper = 0.0;

                // get st range
                if (!boundary->GetUnblockSRange(curr_t, &drive_s_upper,
                                                &drive_s_lower))
                {
                    continue;
                }

                switch (boundary->boundary_type())
                {
                    case STBoundary::BoundaryType::STOP:
                        s_upper_bound = std::fmin(s_upper_bound, drive_s_upper);
                        s_soft_upper_bound =
                                std::fmin(s_soft_upper_bound, drive_s_upper);

                        // check s bound,如果s upper比s lower小，那么s bound
                        // region不合理
                        if (s_upper_bound <= s_lower_bound)
                        {
                            s_upper_bound = s_lower_bound + k_s_bound_error;
                        }

                        if (s_soft_upper_bound <= s_soft_lower_bound)
                        {
                            s_soft_upper_bound =
                                    s_soft_lower_bound + k_s_bound_error;
                        }
                        break;
                    case STBoundary::BoundaryType::YIELD:

                        s_gap = boundary->characteristic_length();

                        if (boundary->is_end_interaction_point_valid())
                        {
                            end_interaction_poi =
                                    boundary->get_end_interaction_poi();

                            if (end_interaction_poi.time() - curr_t < 0.05 &&
                                end_interaction_poi.time() - curr_t > -0.05)
                            {
                                s_gap = std::max(s_gap,
                                                 end_interaction_poi.s_gap());
                            }
                        }

                        yield_upper = drive_s_upper - s_gap;

                        s_upper_bound = std::fmin(s_upper_bound, yield_upper);

                        s_soft_upper_bound =
                                std::fmin(s_soft_upper_bound, drive_s_upper);

                        // check s bound,如果s upper比s lower小，那么s bound
                        // region不合理
                        if (s_upper_bound <= s_lower_bound)
                        {
                            s_upper_bound = s_lower_bound + k_s_bound_error;
                        }

                        if (s_soft_upper_bound <= s_soft_lower_bound)
                        {
                            s_soft_upper_bound =
                                    s_soft_lower_bound + k_s_bound_error;
                        }
                        break;
                    case STBoundary::BoundaryType::FOLLOW:

                        s_gap = boundary->characteristic_length();

                        if (boundary->is_end_interaction_point_valid())
                        {
                            end_interaction_poi =
                                    boundary->get_end_interaction_poi();

                            if (end_interaction_poi.time() - curr_t < 0.05 &&
                                end_interaction_poi.time() - curr_t > -0.05)
                            {
                                s_gap = std::max(s_gap,
                                                 end_interaction_poi.s_gap());
                            }
                        }

                        follow_upper = drive_s_upper - s_gap;

                        s_upper_bound = std::fmin(s_upper_bound, follow_upper);

                        soft_follow_dist =
                                FLAGS_follow_min_distance +
                                std::min(7.0, FLAGS_follow_time_buffer *
                                                      dp_speed_point.v());

                        s_soft_upper_bound =
                                std::fmin(s_soft_upper_bound,
                                          drive_s_upper - soft_follow_dist);

                        // check s bound,如果s upper比s lower小，那么s bound
                        // region不合理
                        if (s_upper_bound <= s_lower_bound)
                        {
                            s_upper_bound = s_lower_bound + k_s_bound_error;
                        }

                        if (s_soft_upper_bound <= s_soft_lower_bound)
                        {
                            s_soft_upper_bound =
                                    s_soft_lower_bound + k_s_bound_error;
                        }
                        break;
                    case STBoundary::BoundaryType::OVERTAKE:
                        s_lower_bound = std::fmax(s_lower_bound, drive_s_lower);
                        s_soft_lower_bound =
                                std::fmax(s_soft_lower_bound,
                                          drive_s_lower + k_overtake_safe_dist);

                        // check s bound,如果s upper比s lower小，那么s bound
                        // region不合理
                        if (s_upper_bound <= s_lower_bound)
                        {
                            s_lower_bound = s_upper_bound - k_s_bound_error;
                        }

                        if (s_soft_upper_bound <= s_soft_lower_bound)
                        {
                            s_soft_lower_bound =
                                    s_soft_upper_bound - k_s_bound_error;
                        }
                        break;
                    default:
                        break;
                }

#if debug_nlp_speed_constraint
                AINFO << "obs id: " << boundary->id() << ", type "
                      << boundary->TypeName(boundary->boundary_type());

                AINFO << "lower s: " << s_lower_bound
                      << " ,upper s: " << s_upper_bound;
#endif
            }

#if debug_nlp_speed_constraint
            AINFO << "t,lower, upper: " << curr_t << " , " << s_lower_bound
                  << " , " << s_upper_bound;

            AINFO << "max dist " << total_length_ << " ,follow s "
                  << FLAGS_follow_min_distance;
#endif

            // check dynamic constraints lower bound
            if (has_dynamic_constraints_lower)
            {
                // follow 生成的drive upper boundary不能小于动力学限制
                s_upper_bound =
                        std::max(s_upper_bound,
                                 dynamic_constraints_lower_point.s() + 0.2);
            }

            // check dynamic constraints upper bound, todo

            // check bound valid
            if (s_lower_bound > s_upper_bound)
            {
                const std::string msg =
                        "s_lower_bound larger than s_upper_bound on STGraph";
                AERROR << msg;

                return Status(ErrorCode::PLANNING_ERROR, msg);
            }

            s_soft_bounds_.emplace_back(s_soft_lower_bound, s_soft_upper_bound);
            s_bounds_.emplace_back(s_lower_bound, s_upper_bound);
        }
    }
    else
    {
        s_bounds_.clear();
        // TODO(Jinyun): move to confs
        for (int i = 0; i < num_of_knots_; ++i)
        {
            double curr_t = i * delta_t_;
            double s_lower_bound = 0.0;
            double s_upper_bound = total_length_;

            for (const STBoundary* boundary : st_graph_data.st_boundaries())
            {
                double s_lower = 0.0;
                double s_upper = 0.0;
                if (!boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower))
                {
                    continue;
                }
                SpeedPoint sp;
                switch (boundary->boundary_type())
                {
                    case STBoundary::BoundaryType::STOP:
                    case STBoundary::BoundaryType::YIELD:
                        s_upper_bound = std::fmin(s_upper_bound, s_upper);
                        break;
                    case STBoundary::BoundaryType::FOLLOW:
                        s_upper_bound = std::fmin(s_upper_bound, s_upper - 8.0);
                        break;
                    case STBoundary::BoundaryType::OVERTAKE:
                        s_lower_bound = std::fmax(s_lower_bound, s_lower);
                        break;
                    default:
                        break;
                }
            }
            if (s_lower_bound > s_upper_bound)
            {
                const std::string msg =
                        "s_lower_bound larger than s_upper_bound on STGraph";
                AERROR << msg;
                return Status(ErrorCode::PLANNING_ERROR, msg);
            }
            s_bounds_.emplace_back(s_lower_bound, s_upper_bound);
        }
    }

    speed_limit_ = st_graph_data.speed_limit();
    cruise_speed_ = reference_line_info_->GetCruiseSpeed();

    return Status::OK();
}

bool PiecewiseJerkSpeedNonlinearOptimizer::CheckSpeedLimitFeasibility()
{
    // a naive check on first point of speed limit
    static constexpr double kEpsilon = 1e-6;
    const double init_speed_limit = speed_limit_.GetSpeedLimitByS(s_init_);

    if (init_speed_limit + kEpsilon < s_dot_init_)
    {
        AERROR << "speed limit [" << init_speed_limit
               << "] lower than initial speed[" << s_dot_init_ << "]";

#if debug_pwj_speed_nlp
        // speed_limit_.debug_string();
#endif

        return false;
    }
    return true;
}

Status PiecewiseJerkSpeedNonlinearOptimizer::SmoothSpeedLimit()
{
    // using piecewise_jerk_path to fit a curve of speed_ref
    // TODO(Hongyi): move smooth configs to gflags
    double delta_s = 2.0;
    std::vector<double> speed_ref;
    for (int i = 0; i < 100; ++i)
    {
        double path_s = i * delta_s;
        double limit = speed_limit_.GetSpeedLimitByS(path_s);
        speed_ref.emplace_back(limit);
    }
    std::array<double, 3> init_state = {speed_ref[0], 0.0, 0.0};
    PiecewiseJerkPathProblem piecewise_jerk_problem(speed_ref.size(), delta_s,
                                                    init_state);
    piecewise_jerk_problem.set_x_bounds(0.0, 50.0);
    piecewise_jerk_problem.set_dx_bounds(-10.0, 10.0);
    piecewise_jerk_problem.set_ddx_bounds(-10.0, 10.0);
    piecewise_jerk_problem.set_dddx_bound(-10.0, 10.0);

    piecewise_jerk_problem.set_weight_x(0.0);
    piecewise_jerk_problem.set_weight_dx(10.0);
    piecewise_jerk_problem.set_weight_ddx(10.0);
    piecewise_jerk_problem.set_weight_dddx(10.0);

    piecewise_jerk_problem.set_x_ref(10.0, std::move(speed_ref));

    if (!piecewise_jerk_problem.Optimize(4000))
    {
        const std::string msg = "Smoothing speed limit failed";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    std::vector<double> opt_x;
    std::vector<double> opt_dx;
    std::vector<double> opt_ddx;

    opt_x = piecewise_jerk_problem.opt_x();
    opt_dx = piecewise_jerk_problem.opt_dx();
    opt_ddx = piecewise_jerk_problem.opt_ddx();
    PiecewiseJerkTrajectory1d smoothed_speed_limit(
            opt_x.front(), opt_dx.front(), opt_ddx.front());

    for (size_t i = 1; i < opt_ddx.size(); ++i)
    {
        double j = (opt_ddx[i] - opt_ddx[i - 1]) / delta_s;
        smoothed_speed_limit.AppendSegment(j, delta_s);
    }

    smoothed_speed_limit_ = smoothed_speed_limit;

    return Status::OK();
}

Status PiecewiseJerkSpeedNonlinearOptimizer::SmoothPathCurvature(
        const PathData& path_data)
{
    // using piecewise_jerk_path to fit a curve of path kappa profile
    // TODO(Jinyun): move smooth configs to gflags
    const auto& cartesian_path = path_data.discretized_path();
    const double delta_s = 0.5;
    std::vector<double> path_curvature;
    for (double path_s = cartesian_path.front().s();
         path_s < cartesian_path.back().s() + delta_s; path_s += delta_s)
    {
        const auto& path_point = cartesian_path.Evaluate(path_s);
        path_curvature.push_back(path_point.kappa());
    }
    const auto& path_init_point = cartesian_path.front();
    std::array<double, 3> init_state = {path_init_point.kappa(),
                                        path_init_point.dkappa(),
                                        path_init_point.ddkappa()};
    PiecewiseJerkPathProblem piecewise_jerk_problem(path_curvature.size(),
                                                    delta_s, init_state);
    piecewise_jerk_problem.set_x_bounds(-1.0, 1.0);
    piecewise_jerk_problem.set_dx_bounds(-10.0, 10.0);
    piecewise_jerk_problem.set_ddx_bounds(-10.0, 10.0);
    piecewise_jerk_problem.set_dddx_bound(-10.0, 10.0);

    piecewise_jerk_problem.set_weight_x(0.0);
    piecewise_jerk_problem.set_weight_dx(10.0);
    piecewise_jerk_problem.set_weight_ddx(10.0);
    piecewise_jerk_problem.set_weight_dddx(10.0);

    piecewise_jerk_problem.set_x_ref(10.0, std::move(path_curvature));

    if (!piecewise_jerk_problem.Optimize(1000))
    {
        const std::string msg = "Smoothing path curvature failed";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    std::vector<double> opt_x;
    std::vector<double> opt_dx;
    std::vector<double> opt_ddx;

    opt_x = piecewise_jerk_problem.opt_x();
    opt_dx = piecewise_jerk_problem.opt_dx();
    opt_ddx = piecewise_jerk_problem.opt_ddx();

    PiecewiseJerkTrajectory1d smoothed_path_curvature(
            opt_x.front(), opt_dx.front(), opt_ddx.front());

    for (size_t i = 1; i < opt_ddx.size(); ++i)
    {
        double j = (opt_ddx[i] - opt_ddx[i - 1]) / delta_s;
        smoothed_path_curvature.AppendSegment(j, delta_s);
    }

    smoothed_path_curvature_ = smoothed_path_curvature;

    return Status::OK();
}

Status PiecewiseJerkSpeedNonlinearOptimizer::OptimizeByQP(
        SpeedData* const speed_data, std::vector<double>* distance,
        std::vector<double>* velocity, std::vector<double>* acceleration,
        PiecewiseJerkSpeedProblem& piecewise_jerk_problem)
{
    // v bound, 如果存在道路限速，并没有考虑进来？限速是非线性的，无法考虑。
    // 车速有时候会超过限速，
    double speed_limit = std::max(max_speed_, s_dot_init_ + 0.1);

    piecewise_jerk_problem.set_dx_bounds(0.0, speed_limit);

    // acc
    piecewise_jerk_problem.set_ddx_bounds(s_ddot_min_, s_ddot_max_);

    // jerk
    piecewise_jerk_problem.set_dddx_bound(s_dddot_min_, s_dddot_max_);

    // s
    piecewise_jerk_problem.set_x_bounds(s_bounds_);

    // TODO(Jinyun): parameter tunnings
    const auto& config =
            config_.piecewise_jerk_nonlinear_speed_optimizer_config();

    piecewise_jerk_problem.set_weight_x(0.0);
    piecewise_jerk_problem.set_weight_dx(0.0);
    piecewise_jerk_problem.set_weight_ddx(config.acc_weight());
    piecewise_jerk_problem.set_weight_dddx(config.jerk_weight());

    // ref line
    std::vector<double> x_ref;
    for (int i = 0; i < num_of_knots_; ++i)
    {
        const double curr_t = i * delta_t_;
        // get path_s
        SpeedPoint sp;
        speed_data->EvaluateByTime(curr_t, &sp);
        x_ref.emplace_back(sp.s());
    }
    piecewise_jerk_problem.set_x_ref(config.ref_s_weight(), std::move(x_ref));

    // 巡航速度，并没有考虑进来？线性的，为何不考虑:

    // 曲率约束，也是非线性的，不好考虑进来

    // Solve the problem
    if (!piecewise_jerk_problem.Optimize())
    {
        const std::string msg =
                "Speed Optimization by Quadratic Programming failed";

        AERROR << msg;

#if debug_pwj_speed_qp
        piecewise_jerk_problem.debugString();

#endif

        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    *distance = piecewise_jerk_problem.opt_x();
    *velocity = piecewise_jerk_problem.opt_dx();
    *acceleration = piecewise_jerk_problem.opt_ddx();
    return Status::OK();
}

Status PiecewiseJerkSpeedNonlinearOptimizer::smooth_lattice_by_qp(
        SpeedData* const speed_data, std::vector<double>* distance,
        std::vector<double>* velocity, std::vector<double>* acceleration,
        PiecewiseJerkSpeedProblem& piecewise_jerk_problem)
{
    // v bound, 如果存在道路限速，并没有考虑进来？限速是非线性的，无法考虑。
    // 车速有时候会超过限速，
    double speed_limit = std::max(max_speed_, s_dot_init_ + 0.1);

    piecewise_jerk_problem.set_dx_bounds(0.0, speed_limit);

    // acc
    piecewise_jerk_problem.set_ddx_bounds(s_ddot_min_, s_ddot_max_);

    // jerk
    piecewise_jerk_problem.set_dddx_bound(s_dddot_min_, s_dddot_max_);

    // s
    piecewise_jerk_problem.set_x_bounds(0.0, total_length_);

    // TODO(Jinyun): parameter tunnings
    const auto& config =
            config_.piecewise_jerk_nonlinear_speed_optimizer_config();

    piecewise_jerk_problem.set_weight_x(0.0);
    piecewise_jerk_problem.set_weight_dx(0.0);
    piecewise_jerk_problem.set_weight_ddx(config.acc_weight());
    piecewise_jerk_problem.set_weight_dddx(config.jerk_weight());

    // ref line
    std::vector<double> x_ref;
    for (int i = 0; i < num_of_knots_; ++i)
    {
        const double curr_t = i * delta_t_;
        // get path_s
        SpeedPoint sp;
        speed_data->EvaluateByTime(curr_t, &sp);
        x_ref.emplace_back(sp.s());
    }
    piecewise_jerk_problem.set_x_ref(config.ref_s_weight(), std::move(x_ref));

    // 巡航速度，并没有考虑进来？线性的，为何不考虑:

    // 曲率约束，也是非线性的，不好考虑进来

    // Solve the problem
    if (!piecewise_jerk_problem.Optimize())
    {
        const std::string msg =
                "Speed Optimization by Quadratic Programming failed.";

        AERROR << msg;

#if debug_pwj_speed_qp
        piecewise_jerk_problem.debugString();

#endif

        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    *distance = piecewise_jerk_problem.opt_x();
    *velocity = piecewise_jerk_problem.opt_dx();
    *acceleration = piecewise_jerk_problem.opt_ddx();
    return Status::OK();
}

Status PiecewiseJerkSpeedNonlinearOptimizer::OptimizeByNLP(
        std::vector<double>* distance, std::vector<double>* velocity,
        std::vector<double>* acceleration)
{
    static std::mutex mutex_tnlp;
    UNIQUE_LOCK_MULTITHREAD(mutex_tnlp);

    // Set optimizer instance
    auto ptr_interface = new PiecewiseJerkSpeedNonlinearIpoptInterface(
            s_init_, s_dot_init_, s_ddot_init_, delta_t_, num_of_knots_,
            total_length_, s_dot_max_, s_ddot_min_, s_ddot_max_, s_dddot_min_,
            s_dddot_max_);

    ptr_interface->set_safety_bounds(s_bounds_);

    // Set weights and reference values
    const auto& config =
            config_.piecewise_jerk_nonlinear_speed_optimizer_config();

    // non linear
    ptr_interface->set_curvature_curve(smoothed_path_curvature_);

    // TODO(Hongyi): add debug_info for speed_limit fitting curve
    // non linear
    ptr_interface->set_speed_limit_curve(smoothed_speed_limit_);

    // TODO(Jinyun): refactor warms start setting API
    if (config.use_warm_start())
    {
        const auto& warm_start_distance = *distance;
        const auto& warm_start_velocity = *velocity;
        const auto& warm_start_acceleration = *acceleration;

        if (warm_start_distance.empty() || warm_start_velocity.empty() ||
            warm_start_acceleration.empty() ||
            warm_start_distance.size() != warm_start_velocity.size() ||
            warm_start_velocity.size() != warm_start_acceleration.size())
        {
            const std::string msg = "Piecewise jerk speed nonlinear optimizer "
                                    "warm start invalid!";
            AERROR << msg;
            return Status(ErrorCode::PLANNING_ERROR, msg);
        }

        std::vector<std::vector<double>> warm_start;
        std::size_t size = warm_start_distance.size();
        for (std::size_t i = 0; i < size; ++i)
        {
            warm_start.emplace_back(std::initializer_list<double>(
                    {warm_start_distance[i], warm_start_velocity[i],
                     warm_start_acceleration[i]}));
        }
        ptr_interface->set_warm_start(warm_start);
    }

    // 0
    if (FLAGS_use_smoothed_dp_guide_line)
    // if (1)
    {
        ptr_interface->set_reference_spatial_distance(*distance);
        // TODO(Jinyun): move to confs
        ptr_interface->set_w_reference_spatial_distance(0.05);
    }
    else
    {
        std::vector<double> spatial_potantial(num_of_knots_, total_length_);
        ptr_interface->set_reference_spatial_distance(spatial_potantial);
        ptr_interface->set_w_reference_spatial_distance(
                config.s_potential_weight());
        // ptr_interface->set_w_reference_spatial_distance(0.0001);
    }

    // 1
    if (FLAGS_use_soft_bound_in_nonlinear_speed_opt)
    {
        ptr_interface->set_soft_safety_bounds(s_soft_bounds_);
        ptr_interface->set_w_soft_s_bound(config.soft_s_bound_weight());
    }

    ptr_interface->set_w_overall_a(config.acc_weight());
    ptr_interface->set_w_overall_j(config.jerk_weight());
    ptr_interface->set_w_overall_centripetal_acc(config.lat_acc_weight());

    ptr_interface->set_reference_speed(cruise_speed_);
    ptr_interface->set_w_reference_speed(config.ref_v_weight());

    Ipopt::SmartPtr<Ipopt::TNLP> problem = ptr_interface;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetIntegerValue("print_level", 0);
    app->Options()->SetIntegerValue("max_iter", 1000);

    Ipopt::ApplicationReturnStatus status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded)
    {
        const std::string msg = "Piecewise jerk speed nonlinear optimizer "
                                "failed during initialization";
        AERROR << msg;

        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    const auto start_timestamp = std::chrono::system_clock::now();
    status = app->OptimizeTNLP(problem);

    const auto end_timestamp = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_timestamp - start_timestamp;

    AINFO << "The optimization problem take time: " << diff.count() * 1000.0
          << " ms.";

    if (status == Ipopt::Solve_Succeeded ||
        status == Ipopt::Solved_To_Acceptable_Level)
    {
        // Retrieve some statistics about the solve
#if debug_pwj_speed_nlp

        Ipopt::Index iter_count = app->Statistics()->IterationCount();
        AINFO << "*** The problem solved in " << iter_count << " iterations!";
        Ipopt::Number final_obj = app->Statistics()->FinalObjective();
        AINFO << "*** The final value of the objective function is "
              << final_obj << '.';

#endif
    }
    else
    {
        const auto& ipopt_return_status =
                IpoptReturnStatus_Name(static_cast<IpoptReturnStatus>(status));
        if (ipopt_return_status.empty())
        {
            AERROR << "Solver ends with unknown failure code: "
                   << static_cast<int>(status);
        }
        else
        {
            AERROR << "Solver failure case is : " << ipopt_return_status;
        }
        const std::string msg =
                "Piecewise jerk speed nonlinear optimizer failed";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    ptr_interface->get_optimization_results(distance, velocity, acceleration);

    return Status::OK();
}

void PiecewiseJerkSpeedNonlinearOptimizer::debug_qp()
{
    AINFO << "debug qp";
    AINFO << "delta t: " << delta_t_;
    AINFO << "path total_length_: " << total_length_;
    AINFO << "total_time_: " << total_time_;
    AINFO << "num_of_knots_: " << num_of_knots_;

    AINFO << "s_init_: " << s_init_;
    AINFO << "s_dot_init_: " << s_dot_init_;
    AINFO << "s_ddot_init_: " << s_ddot_init_;

    AINFO << "s_dot_max_: " << max_speed_;

    AINFO << "s_ddot_min_: " << s_ddot_min_;
    AINFO << "s_ddot_max_: " << s_ddot_max_;

    AINFO << "s_dddot_min_: " << s_dddot_min_;
    AINFO << "s_dddot_max_: " << s_dddot_max_;

    AINFO << "====== s bound";
    for (size_t i = 0; i < s_bounds_.size(); i++)
    {
        AINFO << "t " << i * 0.1 << " ,lower: " << s_bounds_[i].first
              << ", upper: " << s_bounds_[i].second;
    }

    AINFO << "====== speed bound";

    const std::vector<ConstantJerkTrajectory1d>& speed_limit =
            smoothed_speed_limit_.get_segments();
    for (size_t i = 0; i < speed_limit.size(); i++)
    {
        AINFO << "v: " << speed_limit[i].start_position();
    }
}

void PiecewiseJerkSpeedNonlinearOptimizer::debug_nlp()
{
    AINFO << "debug nlp speed";
    AINFO << "delta t: " << delta_t_;
    AINFO << "total_length_: " << total_length_;
    AINFO << "total_time_: " << total_time_;
    AINFO << "num_of_knots_: " << num_of_knots_;

    AINFO << "s_init_: " << s_init_;
    AINFO << "s_dot_init_: " << s_dot_init_;
    AINFO << "s_ddot_init_: " << s_ddot_init_;

    AINFO << "s_dot_max_: " << s_dot_max_;
    AINFO << "s_ddot_min_: " << s_ddot_min_;
    AINFO << "s_ddot_max_: " << s_ddot_max_;

    AINFO << "s_dddot_min_: " << s_dddot_min_;
    AINFO << "s_dddot_max_: " << s_dddot_max_;

    AINFO << "====== s bound";
    for (size_t i = 0; i < s_bounds_.size(); i++)
    {
        AINFO << "lower: " << s_bounds_[i].first
              << ", upper: " << s_bounds_[i].second;
    }

    AINFO << "====== speed bound";

    const std::vector<ConstantJerkTrajectory1d>& speed_limit =
            smoothed_speed_limit_.get_segments();
    for (size_t i = 0; i < speed_limit.size(); i++)
    {
        AINFO << "v: " << speed_limit[i].start_position();
    }

    AINFO << "====== kappa";

    const std::vector<ConstantJerkTrajectory1d>& kappa =
            smoothed_path_curvature_.get_segments();
    for (size_t i = 0; i < kappa.size(); i++)
    {
        AINFO << "kappa: " << kappa[i].start_position();
    }
}

int PiecewiseJerkSpeedNonlinearOptimizer::record_qp_info(
        const std::vector<double>& distance,
        const std::vector<double>& velocity,
        const std::vector<double>& acceleration)
{
    auto* debug = reference_line_info_->mutable_debug();

    // record qp
    auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();
    ptr_speed_plan->set_name("qp_speed");

    if (distance.size() <= 0 || velocity.size() <= 0 ||
        acceleration.size() <= 0)
    {
        return 0;
    }

    common::SpeedPoint tmp_speed_point;

    tmp_speed_point = common::util::PointFactory::ToSpeedPoint(
            distance[0], 0.0, velocity[0], acceleration[0], 0.0);

    common::SpeedPoint* speed_point;

    speed_point = ptr_speed_plan->add_speed_point();
    speed_point->CopyFrom(tmp_speed_point);

    for (int i = 1; i < num_of_knots_; ++i)
    {
        tmp_speed_point = common::util::PointFactory::ToSpeedPoint(
                distance[i], delta_t_ * i, velocity[i], acceleration[i],
                (acceleration[i] - acceleration[i - 1]) / delta_t_);

        speed_point = ptr_speed_plan->add_speed_point();
        speed_point->CopyFrom(tmp_speed_point);
    }

#if 0
    std::array<double, 3> init_state;
    std::array<double, 3> end_condition;

    init_state[0] = 0;
    init_state[1] = 20;
    init_state[2] = 0;

    end_condition[0] = 70;
    end_condition[1] = 10;
    end_condition[2] = 0;
    {
        auto ptr_trajectory1d = std::make_shared<LatticeTrajectory1d>(
                std::shared_ptr<Curve1d>(new QuinticPolynomialCurve1d(
                        init_state, end_condition, 7.0)));

        ptr_trajectory1d->set_target_position(end_condition[0]);
        ptr_trajectory1d->set_target_velocity(end_condition[1]);
        ptr_trajectory1d->set_target_time(7.0);

        for (size_t i = 0; i < 70; i++)
        {
            double time = i * 0.1;

            double acc = ptr_trajectory1d->Evaluate(2, time);

            AINFO << "t: " << time << ", acc: " << acc;
        }
    }

#endif

    return 0;
}

int PiecewiseJerkSpeedNonlinearOptimizer::record_constraints()
{
    auto* debug = reference_line_info_->mutable_debug();

    const SpeedData& brake_speed =
            reference_line_info_->emergency_brake_speed_data();

    if (brake_speed.size() < 1)
    {
        return 0;
    }

    auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();

    ptr_speed_plan->set_name("minimum_jerk_speed_profile");
    ptr_speed_plan->mutable_speed_point()->CopyFrom(
            {brake_speed.begin(), brake_speed.end()});

    planning_internal::STGraphDebug* st_graph_debug =
            debug->mutable_planning_data()->add_st_graph();

    st_graph_debug->set_name(Name());

    // Plot the chosen ST drive boundary.
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name("ST_drive_Boundary");
    boundary_debug->set_type(planning_internal::StGraphBoundaryDebug::
                                     ST_BOUNDARY_TYPE_DRIVABLE_REGION);

    // lower
    double t = 0.0;
    double s_lower = 0.0;
    for (const auto& st_bound_pt : s_bounds_)
    {
        auto point_debug = boundary_debug->add_point();

        s_lower = st_bound_pt.first;

        point_debug->set_t(t);
        point_debug->set_s(s_lower);

        // AINFO << "(" << t << ", " << s_lower << ")";

        t += delta_t_;
    }

    // upper
    int s_bound_size = s_bounds_.size();

    double s_upper = 0.0;

    t -= delta_t_;

    for (int i = s_bound_size - 1; i >= 0; --i)
    {
        auto point_debug = boundary_debug->add_point();

        s_upper = s_bounds_[i].second;

        point_debug->set_t(t);
        point_debug->set_s(s_upper);

        // AINFO << "(" << t << ", " << s_upper << ")";

        t -= delta_t_;
    }

    return 0;
}

int PiecewiseJerkSpeedNonlinearOptimizer::optimize_speed_by_nlp_interface(
        const PathData& path_data, SpeedData* const speed_data,
        std::vector<double>* distance, std::vector<double>* velocity,
        std::vector<double>* acceleration)
{
    if (distance == nullptr || velocity == nullptr || acceleration == nullptr)
    {
        AERROR << "invalid input";

        return 0;
    }

    // check 起点限速情况
    const bool speed_limit_check_status = CheckSpeedLimitFeasibility();

    if (speed_limit_check_status)
    {
        /**  曲率曲线
         * @brief
         * @note
         * @retval
         */
        const auto curvature_smooth_start = std::chrono::system_clock::now();
        const auto path_curvature_smooth_status =
                SmoothPathCurvature(path_data);

        const auto curvature_smooth_end = std::chrono::system_clock::now();

        std::chrono::duration<double> curvature_smooth_diff =
                curvature_smooth_end - curvature_smooth_start;

        AINFO << "path curvature smoothing time takes "
              << curvature_smooth_diff.count() * 1000.0 << " ms";

        if (!path_curvature_smooth_status.ok())
        {
            speed_data->clear();

            AERROR << "path kappa smooth fail";
            return 0;
        }

        /**  限速曲线
         * @brief
         * @note
         * @retval
         */
        const auto speed_limit_smooth_start = std::chrono::system_clock::now();

        const auto speed_limit_smooth_status = SmoothSpeedLimit();

        const auto speed_limit_smooth_end = std::chrono::system_clock::now();

        std::chrono::duration<double> speed_limit_smooth_diff =
                speed_limit_smooth_end - speed_limit_smooth_start;

        AINFO << "speed limit smoothing for time takes "
              << speed_limit_smooth_diff.count() * 1000.0 << " ms";

        if (!speed_limit_smooth_status.ok())
        {
            speed_data->clear();

            AERROR << "speed limit smooth fail";
            return 0;
        }

        // Record speed_constraint
        record_nlp_info();

        const auto nlp_start = std::chrono::system_clock::now();

        const auto nlp_smooth_status =
                OptimizeByNLP(distance, velocity, acceleration);

#if debug_pwj_speed_qp
        debug_nlp();
#endif

        const auto nlp_end = std::chrono::system_clock::now();
        std::chrono::duration<double> nlp_diff = nlp_end - nlp_start;

        AINFO << "print_speed_nlp_optimization:"
              << "(" << nlp_diff.count() * 1000 << ","
              << ")";

        if (!nlp_smooth_status.ok())
        {
            speed_data->clear();

            AERROR << "nlp speed optimizer fail";
            return 0;
        }
    }

    return 0;
}

int PiecewiseJerkSpeedNonlinearOptimizer::record_nlp_info()
{
    auto* debug = reference_line_info_->mutable_debug();

    int st_graph_size = debug->mutable_planning_data()->st_graph_size();
    if (st_graph_size < 1)
    {
        return 0;
    }

    planning_internal::STGraphDebug* st_graph_debug =
            debug->mutable_planning_data()->mutable_st_graph(st_graph_size - 1);

    const std::vector<std::pair<double, double>>& speed_limit_pts =
            speed_limit_.speed_limit_points();

    for (int i = 0; i < speed_limit_pts.size(); ++i)
    {
        double s = speed_limit_pts[i].first;

        double v = smoothed_speed_limit_.Evaluate(0, s);

        common::SpeedPoint* speed_point = st_graph_debug->add_speed_limit();
        speed_point->set_s(s);
        speed_point->set_v(v);
    }

    return 0;
}

void PiecewiseJerkSpeedNonlinearOptimizer::get_ego_lon_state(
        double* s, double* v, const double ego_init_v, const double acc,
        const double time)
{
    *s = ego_init_v * time + 0.5 * acc * time * time;

    *v = ego_init_v + acc * time;

    return;
}

void PiecewiseJerkSpeedNonlinearOptimizer::get_obs_lon_state(
        double* s, const double obs_init_v, const double obs_init_s,
        const double time)
{
    *s = obs_init_s + obs_init_v * time;
    return;
}

int PiecewiseJerkSpeedNonlinearOptimizer::is_follow_state_valid(
        double* time_headway, double* ttc, const double ego_s,
        const double ego_v, const double obs_s, const double obs_v)
{
    double delta_s = obs_s - ego_s;

    if (delta_s < 0.2)
    {
        *time_headway = 0.0;
        *ttc = 0.0;
        return 0;
    }

    if (ego_v < 0.1)
    {
        *time_headway = 1000.0;
        *ttc = 1000.0;
        return 0;
    }
    else
    {
        *time_headway = delta_s / ego_v;

        if (ego_v > obs_v)
        {
            *ttc = delta_s / (ego_v - obs_v);
        }
        else
        {
            *ttc = 1000.0;
        }
    }

    return 0;
}

void PiecewiseJerkSpeedNonlinearOptimizer::debug_speed_lattice_profile(
        const speed_lattice_cost& cost)
{
    AINFO << "start point gap: " << cost.start_point_lon_gap
          << " ,time headway: " << cost.start_point_time_headway;
    AINFO << "close point gap: " << cost.min_lon_gap
          << " ,time headway: " << cost.min_time_headway;
    AINFO << "end point gap: " << cost.end_point_lon_gap
          << " ,time headway: " << cost.end_point_time_headway;

    AINFO << "acc: " << cost.sampling_acc << " ,time: " << cost.sampling_time;
    return;
}

}  // namespace planning
}  // namespace apollo
