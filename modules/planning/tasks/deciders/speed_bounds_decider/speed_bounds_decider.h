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
 * @file
 **/

#pragma once

#include <memory>

#include "modules/planning/common/frame.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo
{
namespace planning
{

enum class speed_bounds_decider_stage
{
    SPEED_BOUNDS_PRIORI_DECISION = 0,
    SPEED_BOUNDS_POSTERIOR_DECISION  =1 
};

// s以path start point为基准；
// t 以当前帧规划的时间戳为基准，path start point的relative time可能是0.1秒。
// 预测和规划都使用了relative time，看来base time stamp要对齐？

// 这个API调用了2次，先决策，后决策
// SpeedBoundsDecider -> DP -> speed decider -> SpeedBoundsDecider
// 按照这个流程，确定决策结果

// 该类，只根据决策结果，来生成st boundary
class SpeedBoundsDecider : public Decider
{
public:
    SpeedBoundsDecider(const TaskConfig& config,
                       const std::shared_ptr<DependencyInjector>& injector);

private:
    common::Status Process(
            Frame* const frame,
            ReferenceLineInfo* const reference_line_info) override;

    double SetSpeedFallbackDistance(PathDecision* const path_decision);

    void RecordSTGraphDebug(
            const StGraphData& st_graph_data,
            planning_internal::STGraphDebug* st_graph_debug) const;

private:
    SpeedBoundsDeciderConfig speed_bounds_config_;

    speed_bounds_decider_stage  decision_stage_;
};

}  // namespace planning
}  // namespace apollo
