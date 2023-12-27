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

#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/proto/st_drivable_boundary.pb.h"
#include "modules/planning/proto/task_config.pb.h"
#include "modules/planning/tasks/optimizers/path_time_heuristic/st_graph_point.h"

namespace apollo
{
namespace planning
{

//由于某些因素的存在，导致cost无穷大,记录类型
enum class edge_cost_inf_type
{
    collision_with_obs = 0,
    out_of_acc_limit = 1,
    negative_speed = 2,
    no_father_node = 3,
    father_node_cost_is_inf = 4,
};

class DpStCost
{
public:
    DpStCost(const DpStSpeedOptimizerConfig& config, const double total_t,
             const double total_s,
             const std::vector<const Obstacle*>& obstacles,
             const STDrivableBoundary& st_drivable_boundary,
             const common::TrajectoryPoint& init_point,
             const SLBoundary& adc_sl_bound);

    double GetObstacleCost(const StGraphPoint& point);

    double GetSpatialPotentialCost(const StGraphPoint& point);

    double GetReferenceCost(const STPoint& point,
                            const STPoint& reference_point) const;

    double GetSpeedCost(const STPoint& first, const STPoint& second,
                        const double speed_limit,
                        const double cruise_speed) const;

    double GetAccelCostByTwoPoints(const double pre_speed, const STPoint& first,
                                   const STPoint& second);
    double GetAccelCostByThreePoints(const STPoint& first,
                                     const STPoint& second,
                                     const STPoint& third);

    double GetJerkCostByTwoPoints(const double pre_speed, const double pre_acc,
                                  const STPoint& pre_point,
                                  const STPoint& curr_point);
    double GetJerkCostByThreePoints(const double first_speed,
                                    const STPoint& first_point,
                                    const STPoint& second_point,
                                    const STPoint& third_point);

    double GetJerkCostByFourPoints(const STPoint& first, const STPoint& second,
                                   const STPoint& third, const STPoint& fourth);

    double GetAccelByTwoPoint(const double left_point_speed,
                              const common::SpeedPoint& left_point,
                              const common::SpeedPoint& right_point);

    double GetJerkByTwoPoint(const double left_point_speed,
                             const double left_point_acc,
                             const common::SpeedPoint& left_point,
                             const common::SpeedPoint& right_point);

    double GetAccelCost(const double accel);
    
    double JerkCost(const double jerk);

    double get_speed_cost(const double cur_v, const STPoint& first,
                          const STPoint& second, const double speed_limit,
                          const double cruise_speed) const;

private:


    // keep clear zone process 
    void AddToKeepClearRange(const std::vector<const Obstacle*>& obstacles);
    
    static void SortAndMergeRange(
            std::vector<std::pair<double, double>>* keep_clear_range_);
    
    bool InKeepClearRange(double s) const;

    const DpStSpeedOptimizerConfig& config_;
    const std::vector<const Obstacle*>& obstacles_;

    // st_bounds_decider生成的boundary，有较多bug，暂时不使用。
    // 目前，决策结果完全依赖于dp?
    STDrivableBoundary st_drivable_boundary_;

    const common::TrajectoryPoint& init_point_;
    const SLBoundary &adc_sl_bound_;

    double unit_t_ = 0.0;

    // path 长度，在有障碍物阻挡时，会比较短。一般地，120米。
    double total_s_ = 0.0;

    // [obs_id, obs index in boundary_cost_ list]
    std::unordered_map<std::string, int> boundary_map_;

    // obs_number * dimension_t
    // std::pair<double, double>: [s upper, s lower], st boundary上下界
    std::vector<std::vector<std::pair<double, double>>> boundary_cost_;

    std::vector<std::pair<double, double>> keep_clear_range_;

    // acc cost table,
    // 每一个acc值的cost都不同, 
    // https://qwybs7wggx.feishu.cn/docx/Ll3edQHMGoRG0zxM978cNFw9nZd
    std::array<double, 200> accel_cost_;

    // jerk cost table, 可以查表，也可以计算生成，查表性能好一些
    std::array<double, 400> jerk_cost_;
};

}  // namespace planning
}  // namespace apollo
