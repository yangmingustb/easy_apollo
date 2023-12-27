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
#include "modules/planning/traffic_rules/destination.h"

#include <memory>
#include <vector>

#include "modules/map/proto/map_lane.pb.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"

namespace apollo
{
namespace planning
{
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

Destination::Destination(const TrafficRuleConfig& config,
                         const std::shared_ptr<DependencyInjector>& injector) :
    TrafficRule(config, injector)
{
}

Status Destination::ApplyRule(Frame* frame,
                              ReferenceLineInfo* const reference_line_info)
{
    CHECK_NOTNULL(frame);
    CHECK_NOTNULL(reference_line_info);

    MakeDecisions(frame, reference_line_info);

    return Status::OK();
}

/**
 * @brief: build stop decision
 */
int Destination::MakeDecisions(Frame* frame,
                               ReferenceLineInfo* const reference_line_info)
{
    CHECK_NOTNULL(frame);
    CHECK_NOTNULL(reference_line_info);

    if (!frame->is_near_destination())
    {
        return 0;
    }

    const auto& routing = frame->local_view().routing;
    if (routing->routing_request().waypoint_size() < 2)
    {
        AERROR << "routing_request has no end";
        return -1;
    }

    const auto& reference_line = reference_line_info->reference_line();
    // end point
    // 如果是一个循环路线，dest_sl.s 可能为0，使用ref line投影计算有bug
    // 如果是u型弯， desitnation 也可能很小，一般而言，destination sl的l在0附近。
    const auto& routing_end = *(routing->routing_request().waypoint().rbegin());

    common::SLPoint dest_sl;
    reference_line.XYToSL(routing_end.pose(), &dest_sl);

#if debug_destination
    AINFO << "dest s: " << dest_sl.s() << ", l: " << dest_sl.l();

#endif

    const auto& adc_sl = reference_line_info->AdcSlBoundary();
    const auto& dest = injector_->planning_context()
                               ->mutable_planning_status()
                               ->destination();

    // 如果车辆超过destination，且没有做出decision，那么要报错
    double dest_lat_dist_max = 6.0;

    // 这里主要通过比较dest 和ego在 ref line上的投影，这里有bug，需要refact
    if (std::fabs(dest_sl.l()) < dest_lat_dist_max && dest_sl.s() > 0.0)
    {
        if (adc_sl.start_s() > dest_sl.s() && !dest.has_passed_destination())
        {
            AERROR << "Destination at back, but we have not reached "
                      "destination "
                      "yet, should stop";
            return 0;
        }
    }

    const std::string stop_wall_id = FLAGS_destination_obstacle_id;
    const std::vector<std::string> wait_for_obstacle_ids;

    // 0
    if (FLAGS_enable_scenario_pull_over)
    {
        const auto& pull_over_status =
                injector_->planning_context()->planning_status().pull_over();
        if (pull_over_status.has_position() &&
            pull_over_status.position().has_x() &&
            pull_over_status.position().has_y())
        {
            // build stop decision based on pull-over position
            ADEBUG << "BuildStopDecision: pull-over position";
            common::SLPoint pull_over_sl;
            reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);

            const double stop_line_s = pull_over_sl.s() +
                                       VehicleConfigHelper::GetConfig()
                                               .vehicle_param()
                                               .front_edge_to_center() +
                                       config_.destination().stop_distance();
            util::BuildStopDecision(
                    stop_wall_id, stop_line_s,
                    config_.destination().stop_distance(),
                    StopReasonCode::STOP_REASON_PULL_OVER,
                    wait_for_obstacle_ids,
                    TrafficRuleConfig::RuleId_Name(config_.rule_id()), frame,
                    reference_line_info);
            return 0;
        }
    }

    // build stop decision
    AINFO << "BuildStopDecision: destination, end s: " << routing_end.s();

    double dist_to_end =  dest_sl.s() - adc_sl.end_s();
    AINFO << "BuildStopDecision: dist to destination: " << dist_to_end;

    // const double dest_lane_s =
    //         std::fmax(0.0, routing_end.s() - FLAGS_virtual_stop_wall_length -
    //                                config_.destination().stop_distance());

    const double dest_lane_s = std::fmax(0.0, routing_end.s());
    util::BuildStopDecision(stop_wall_id, routing_end.id(), dest_lane_s,
                            config_.destination().stop_distance(),
                            StopReasonCode::STOP_REASON_DESTINATION,
                            wait_for_obstacle_ids,
                            TrafficRuleConfig::RuleId_Name(config_.rule_id()),
                            frame, reference_line_info);

    return 0;
}

}  // namespace planning
}  // namespace apollo
