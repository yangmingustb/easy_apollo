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

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo
{
namespace planning
{

#define debug_path_lane_borrow_decider (1)

enum class no_need_lane_borrow_reason
{
    none = 0,
    prior_lane_no_blocking_obs = 1,
    neighbour_lane_has_obs,
    neighbour_lane_has_low_speed_obs,
    neighbour_lane_has_close_obs,
    no_neighbour_lane_or_break_traffic_rule,
};

enum class stop_enter_prior_lane_reason
{
    none = 0,
    // lane borrow time不能太短，一帧借道一帧不借道是不允许的
    lane_borrow_time_is_short,
    prior_lane_blocking_obs,
    prior_lane_solid_lane_line,
};

// 单条参考线，才会有借道行为;
// 目前的实现：多参考线不会lane borrow，only lane change;
// 多参考线是route变道
// 单参考线是obs阻挡引起变道
class PathLaneBorrowDecider : public Decider
{
public:
    PathLaneBorrowDecider(const TaskConfig& config,
                          const std::shared_ptr<DependencyInjector>& injector);

    static bool has_manual_lane_borrow_decision(
           const LaneBorrowStatus& decider);

    static bool has_lane_borrow_decision_by_turtle_car(
           const LaneBorrowStatus& decider);

    static bool has_left_lane_borrow_decision_by_turtle_car(
            const LaneBorrowStatus& decider);

    static bool has_left_lane_borrow_decision(
            const LaneBorrowStatus& decider);

private:
    common::Status Process(Frame* frame,
                           ReferenceLineInfo* reference_line_info) override;

    bool IsNecessaryToBorrowLane(const Frame& frame,
                                 const ReferenceLineInfo& reference_line_info);

    bool update_lane_borrow_decision_by_manual(const Frame& frame,
                                 const ReferenceLineInfo& reference_line_info);

    bool update_lane_borrow_decision_by_low_speed_obs(const Frame& frame,
                                 const ReferenceLineInfo& reference_line_info);

    bool HasSingleReferenceLine(const Frame& frame);

    bool IsWithinSidePassingSpeedADC(const Frame& frame);

    bool IsLongTermBlockingObstacle();

    bool IsBlockingObstacleWithinDestination(
            const ReferenceLineInfo& reference_line_info);

    bool IsBlockingObstacleFarFromIntersection(
            const ReferenceLineInfo& reference_line_info);

    bool IsSidePassableObstacle(const ReferenceLineInfo& reference_line_info);

    void CheckLaneBorrow(const ReferenceLineInfo& reference_line_info,
                         bool* left_neighbor_lane_borrowable,
                         bool* right_neighbor_lane_borrowable);
};

}  // namespace planning
}  // namespace apollo
