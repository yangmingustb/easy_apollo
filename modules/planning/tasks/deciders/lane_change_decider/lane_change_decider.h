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

#include <list>
#include <memory>
#include <string>

#include "modules/map/pnc_map/route_segments.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_status.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo
{
namespace planning
{

#define debug_lane_change_decider (0)


struct LaneChangeRequest
{
    int lane_id;
    LaneChangeRequestType type;
    double max_drive_dist;
    double max_drive_speed;
};

enum class NoLaneChangeType
{
    none,
    solid_lane_line,
    // 花坛，护栏，锥桶，都是路障，所以不能换道
    roadblock,
    // 不是邻居车道
    not_neighbour_lane,
    // 因为其他障碍物的存在，不安全
    not_safe
};

struct LaneChangeResponse
{
    int lane_id;
    NoLaneChangeType type;
    double max_drive_dist;
    double max_drive_speed;
};

// 这里并不会生成换道请求，而是更新换道相关的状态，ChangeLaneStatus
// 换道相关的请求：开始换道，停止换道。这些在哪里计算？
// 只在lane follow场景中会有这个计算任务
class LaneChangeDecider : public Decider
{
public:
    LaneChangeDecider(const TaskConfig& config,
                      const std::shared_ptr<DependencyInjector>& injector);

    /**
     * @brief A static function to check if the ChangeLanePath type of reference
     * line is safe or if current reference line is safe to deviate away and
     * come back
     */
    static bool IsClearToChangeLane(ReferenceLineInfo* reference_line_info);

    /**
     * @brief A static function to estimate if an obstacle in certain range in
     * front of ADV blocks too much space perception behind itself by beam
     * scanning
     * @param search_beam_length is the length of scanning beam
     * @param search_beam_radius_intensity is the resolution of scanning
     * @param search_range is the scanning range centering at ADV heading
     * @param is_block_angle_threshold is the threshold to tell how big a block
     *        angle range is perception blocking
     */
    static bool IsPerceptionBlocked(
            const ReferenceLineInfo& reference_line_info,
            const double search_beam_length,
            const double search_beam_radius_intensity,
            const double search_range, const double is_block_angle_threshold);

    static void UpdatePreparationDistance(
            const bool is_opt_succeed, const Frame* frame,
            const ReferenceLineInfo* const reference_line_info,
            PlanningContext* planning_context);

private:

    common::Status Process(
            Frame* frame,
            ReferenceLineInfo* const current_reference_line_info) override;

public:
    static bool HysteresisFilter(const double obstacle_distance,
                                 const double safe_distance,
                                 const double distance_buffer,
                                 const bool is_obstacle_blocking);

    void UpdateStatus(ChangeLaneStatus::Status status_code,
                      const std::string& path_id);

    void UpdateStatus(double timestamp, ChangeLaneStatus::Status status_code,
                      const std::string& path_id);

    // 改变ref line的顺序，目标车道优先
    void PrioritizeChangeLane(
            const bool is_prioritize_change_lane,
            std::list<ReferenceLineInfo>* reference_line_info);

    static void update_ref_line_sequence(
            const bool is_prioritize_change_lane,
            std::list<ReferenceLineInfo>* reference_line_info);

    void RemoveChangeLane(
            std::list<ReferenceLineInfo>* reference_line_info) const;

    std::string GetCurrentPathId(
            const std::list<ReferenceLineInfo>& reference_line_info) const;
};

}  // namespace planning
}  // namespace apollo
