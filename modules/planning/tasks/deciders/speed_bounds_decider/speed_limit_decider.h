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
 *   @file
 **/

#pragma once

#include <string>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/proto/task_config.pb.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo
{
namespace planning
{

#define enable_brake_prepare_before_curve_lane (0)
#define brake_prepare_dist_before_curve_lane (10.0)
#define curve_lane_max_speed (4.0)

// 在生成限速的时候，要考虑当前速度，
// 下匝道时，车速100，然后限速40，如果直接将这个限速输入给pwj speed
// planner，那么无法生成速度曲线。
// 所以，为了车辆舒适度，以及考虑到求解能力，需要增加一个平滑操作

// 限速来源：
// path kappa
// map speed limit
// caution

class SpeedLimitDecider
{
public:
    SpeedLimitDecider(const SpeedBoundsDeciderConfig& config,
                      const ReferenceLine& reference_line,
                      const PathData& path_data);

    virtual ~SpeedLimitDecider() = default;

    virtual common::Status GetSpeedLimits(
            const IndexedList<std::string, Obstacle>& obstacles,
            SpeedLimit* const speed_limit_data,
            const double speed_limit_by_scenario) const;

    int smooth_speed_limit(SpeedLimit* speed_limit_data, double adc_speed);

private:
    FRIEND_TEST(SpeedLimitDeciderTest, get_centric_acc_limit);
    double GetCentricAccLimit(const double kappa) const;

    void GetAvgKappa(const std::vector<common::PathPoint>& path_points,
                     std::vector<double>* kappa) const;

private:
    const SpeedBoundsDeciderConfig& speed_bounds_config_;
    const ReferenceLine& reference_line_;
    const PathData& path_data_;
    const apollo::common::VehicleParam& vehicle_param_;
};

}  // namespace planning
}  // namespace apollo
