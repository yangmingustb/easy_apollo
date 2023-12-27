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

#include <memory>
#include <string>

#include "modules/planning/traffic_rules/traffic_rule.h"

namespace apollo
{
namespace planning
{
/**
 * This class creates a virtual obstacle for each clear area region.
 * 禁停区，车辆不可以停车。交通路口一般都是禁止停车区，为了防止车辆停在这里，会生成一个虚拟障碍物.
 *
 * 禁止停车区会生成一个虚拟obs，如果前方不能走，那么车辆就停在这个虚拟障碍物后方.
 * 生成st图的时候，如何确定t？
 */
class KeepClear : public TrafficRule
{
public:
    KeepClear(const TrafficRuleConfig& config,
              const std::shared_ptr<DependencyInjector>& injector);
    virtual ~KeepClear() = default;

    common::Status ApplyRule(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info);

private:
    bool IsCreeping(const double pnc_junction_start_s,
                    const double adc_front_edge_s) const;

    bool BuildKeepClearObstacle(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info,
                                const std::string& virtual_obstacle_id,
                                const double keep_clear_start_s,
                                const double keep_clear_end_s);

private:
    static constexpr char const* KEEP_CLEAR_VO_ID_PREFIX = "KC_";
    static constexpr char const* KEEP_CLEAR_JUNCTION_VO_ID_PREFIX = "KC_JC_";
};

}  // namespace planning
}  // namespace apollo
