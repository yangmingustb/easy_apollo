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
 * @file piecewise_jerk_speed_nonlinear_optimizer.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "modules/planning/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning/tasks/optimizers/speed_optimizer.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"


namespace apollo
{
namespace planning
{

struct speed_lattice_cost
{
    // start point info
    double start_point_lon_gap;
    double start_point_time_headway;

    // end point info
    double end_point_lon_gap;
    double end_point_time_headway;

    // close point info
    double min_lon_gap;
    double min_time_headway;


    double sampling_acc;
    double sampling_time;
    
    double speed_limit;

    double time_headway_cost;
    double sampling_acc_cost;

    double total_cost;
};


// nlp优化，how:
// 关于曲率限速，是生成speed limit好，还是放到优化模型中处理？先生成speed
// limit,再作为nlp 的speed bound.
class PiecewiseJerkSpeedNonlinearOptimizer : public SpeedOptimizer
{
public:
    explicit PiecewiseJerkSpeedNonlinearOptimizer(const TaskConfig& config);

    virtual ~PiecewiseJerkSpeedNonlinearOptimizer() = default;

    void debug_nlp();

    void debug_qp();

    int record_qp_info(const std::vector<double>& distance,
                       const std::vector<double>& velocity,
                       const std::vector<double>& acceleration);

    int record_nlp_info();

    int record_constraints();

    int speed_lattice_process(SpeedData* const speed_data,
                              const double speed_limit);

    int estimate_min_lon_gap_for_follow_decision(
            speed_lattice_cost* speed_cost, const Obstacle& obstacle,
            const SpeedData& adc_speed_data);

    double estimate_speed_limit_cost(const SpeedData& adc_speed_data);

    int post_process(SpeedData* const speed_data);

    bool check_constant_speed_profile_is_nice(
            const SpeedData& speed_data, const speed_lattice_cost& speed_cost);

    bool check_speed_up_profile_is_nice(const SpeedData& speed_data,
                                        const speed_lattice_cost& speed_cost);

    bool check_speed_down_profile_is_nice(const SpeedData& speed_data,
                                        const speed_lattice_cost& speed_cost);

    void debug_speed_lattice_profile(const speed_lattice_cost &cost);

private:
    common::Status Process(const PathData& path_data,
                           const common::TrajectoryPoint& init_point,
                           SpeedData* const speed_data) override;

    common::Status SetUpStatesAndBounds(const PathData& path_data,
                                        const SpeedData& speed_data);

    bool CheckSpeedLimitFeasibility();

    common::Status SmoothSpeedLimit();

    common::Status SmoothPathCurvature(const PathData& path_data);


    /**
     * @brief         J = acc^2 + jerk^2
     * 
     * 
     * qp， nlp在这里有什么区别:
     * 
     * qp: 限速、巡航速度，弯道限速，没有考虑
     * @note   
     * @param  speed_data: 
     * @param  distance: 
     * @param  velocity: 
     * @param  acceleration: 
     * @retval 
     */
    common::Status OptimizeByQP(
            SpeedData* const speed_data, std::vector<double>* distance,
            std::vector<double>* velocity, std::vector<double>* acceleration,
            PiecewiseJerkSpeedProblem& piecewise_jerk_problem);

    common::Status smooth_lattice_by_qp(
            SpeedData* const speed_data, std::vector<double>* distance,
            std::vector<double>* velocity, std::vector<double>* acceleration,
            PiecewiseJerkSpeedProblem& piecewise_jerk_problem);

    /**
     * @brief  限速、巡航速度，弯道限速，如何处理：
     *                     J =  f
     *        subject to:  g
     * 
     * f中有弯道限速
     * @note   
     * @param  distance: 
     * @param  velocity: 
     * @param  acceleration: 
     * @retval 
     */
    common::Status OptimizeByNLP(std::vector<double>* distance,
                                 std::vector<double>* velocity,
                                 std::vector<double>* acceleration);

    int optimize_speed_by_nlp_interface(const PathData& path_data,
                                        SpeedData* const speed_data,
                                        std::vector<double>* distance,
                                        std::vector<double>* velocity,
                                        std::vector<double>* acceleration);

    void get_ego_lon_state(double* s, double* v, const double ego_init_v,
                           const double acc, const double time);

    void get_obs_lon_state(double* s, const double obs_init_v,
                           const double obs_init_s, const double time);

    int is_follow_state_valid(double* time_headway, double* ttc,
                              const double ego_s, const double ego_v,
                              const double obs_s, const double obs_v);

    // st problem dimensions
    double delta_t_ = 0.0;
    double total_length_ = 0.0;
    double total_time_ = 0.0;
    int num_of_knots_ = 0;

    // st initial values
    double s_init_ = 0.0;
    double s_dot_init_ = 0.0;
    double s_ddot_init_ = 0.0;

    // st states dynamically feasibility bounds
    double s_dot_max_ = 0.0;
    double s_ddot_min_ = 0.0;
    double s_ddot_max_ = 0.0;
    double s_dddot_min_ = 0.0;
    double s_dddot_max_ = 0.0;

    // st safety bounds
    // 即使在st bounds
    // decider生成了一个可行驶区域，但是qp并没有使用，而是使用dp决策结果和障碍物边界，
    // 来生成的st_bound. 这里s_bounds_并不是st decider 生成
    // 每一个t, 都有lower s, upper s, 
    std::vector<std::pair<double, double>> s_bounds_;

    // 更保守的边界
    std::vector<std::pair<double, double>> s_soft_bounds_;

    // speed limits
    SpeedLimit speed_limit_;

    // s-v curve
    PiecewiseJerkTrajectory1d smoothed_speed_limit_;

    // smoothed path curvature profile as a function of traversal distance
    // s-kappa curve
    PiecewiseJerkTrajectory1d smoothed_path_curvature_;

    // reference speed profile
    // 没有模块生成，也没有模块使用
    SpeedData reference_speed_data_;

    // reference cruise speed
    double cruise_speed_;

    double max_speed_;

    double veh_rear_to_front_;
};

}  // namespace planning
}  // namespace apollo
