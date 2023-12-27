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

#include "modules/planning/traffic_rules/backside_vehicle.h"
#include "modules/planning/common/sl_boundary.h"

namespace apollo
{
namespace planning
{
using apollo::common::Status;

BacksideVehicle::BacksideVehicle(const TrafficRuleConfig& config) :
    TrafficRule(config)
{
}

void BacksideVehicle::make_backside_obstacle_decision_in_lane_keeping(
        const SLBoundary& adc_sl_boundary, PathDecision* path_decision)
{
    ObjectDecisionType ignore;
    ignore.mutable_ignore();

    const double adc_length_s =
            adc_sl_boundary.end_s() - adc_sl_boundary.start_s();

    for (const auto* obstacle : path_decision->obstacles().Items())
    {
        // 前方obs，不可忽略
        if (obstacle->PerceptionSLBoundary().end_s() >=
                    adc_sl_boundary.end_s() ||
            obstacle->IsCautionLevelObstacle())
        {
            // don't ignore such vehicles.
            continue;
        }

        // 没有ref line投影，因为在ref line模块已经忽略，这里继续增加一次ignore
        if (obstacle->reference_line_st_boundary().IsEmpty())
        {
            path_decision->AddLongitudinalDecision(
                    "backside_vehicle/no-st-region", obstacle->Id(), ignore);

            path_decision->AddLateralDecision("backside_vehicle/no-st-region",
                                              obstacle->Id(), ignore);
            continue;
        }

        // Ignore the car comes from back of ADC
        if (obstacle->reference_line_st_boundary().min_s() < -adc_length_s)
        {
            path_decision->AddLongitudinalDecision(
                    "backside_vehicle/st-min-s < adc", obstacle->Id(), ignore);

            path_decision->AddLateralDecision("backside_vehicle/st-min-s < adc",
                                              obstacle->Id(), ignore);
            continue;
        }

        // back side
        if (obstacle->PerceptionSLBoundary().start_s() <
            adc_sl_boundary.start_s())
        {
            const double lane_boundary =
                    config_.backside_vehicle().backside_lane_width();

            // left side or right side,
            // 横向距离较大，说明后车有超车意图，不能忽略
            if (obstacle->PerceptionSLBoundary().start_l() > lane_boundary ||
                obstacle->PerceptionSLBoundary().end_l() < -lane_boundary)
            {
                continue;
            }

            bool is_lateral_overlap = is_lateral_overlap_for_sl_box(
                    obstacle->PerceptionSLBoundary(), adc_sl_boundary);

            if (is_lateral_overlap)
            {
                path_decision->AddLongitudinalDecision(
                        "backside_vehicle/sl < adc.end_s", obstacle->Id(),
                        ignore);

                path_decision->AddLateralDecision(
                        "backside_vehicle/sl < adc.end_s", obstacle->Id(),
                        ignore);
            }
            continue;
        }
    }

    return;
}

Status BacksideVehicle::ApplyRule(Frame* const,
                                  ReferenceLineInfo* const reference_line_info)
{
    auto* path_decision = reference_line_info->path_decision();
    const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();

    // The lane keeping reference line.
    // 如果处于lane change 开始状态, 那么IsOnSegment()可能是false
    if (reference_line_info->Lanes().IsOnSegment())
    {
        make_backside_obstacle_decision_in_lane_keeping(adc_sl_boundary,
                                                        path_decision);
    }

    return Status::OK();
}

bool is_ignore_backside_obstacle(bool is_lane_change_path,
                                 bool is_adc_on_reference_line,
                                 const Obstacle& obstacle,
                                 const SLBoundary& adc_sl_boundary,
                                 const ReferenceLine& reference_line)
{


    // if adc is on the road, and obstacle behind adc, ignore

    bool is_lane_keep = is_adc_on_reference_line && !is_lane_change_path;

    const SLBoundary& obs_sl = obstacle.PerceptionSLBoundary();

    if (is_lane_keep)
    {
        if (obstacle.speed() < 0.1)
        {
            if (obs_sl.end_s() + 10.0 < adc_sl_boundary.start_s())
            {
                return true;
            }
        }

        // 完全重叠
        if (obs_sl.start_l() >= adc_sl_boundary.start_l() &&
            obs_sl.end_l() <= adc_sl_boundary.end_l())
        {
            return true;
        }

        //

        if (obs_sl.start_l() <= adc_sl_boundary.start_l() &&
            obs_sl.end_l() >= adc_sl_boundary.end_l())
        {
            return true;
        }

        double obs_middle_l = (obs_sl.end_l() + obs_sl.start_l()) / 2.0;

        if (obs_middle_l > adc_sl_boundary.start_l() &&
            obs_middle_l <= adc_sl_boundary.end_l())
        {
            return true;
        }

        if (obs_middle_l < adc_sl_boundary.end_l() &&
            obs_middle_l >= adc_sl_boundary.start_l())
        {
            return true;
        }
    }

    return false;
}

}  // namespace planning
}  // namespace apollo
