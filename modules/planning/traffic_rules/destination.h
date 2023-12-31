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

#include "modules/common/proto/geometry.pb.h"
#include "modules/planning/traffic_rules/traffic_rule.h"

namespace apollo
{
namespace planning
{

#define debug_destination (0)
/**
 * This class decides whether we should stop for destination.
 * situation. 根据destination 生成virtual obstacle.
 * 
 */
class Destination : public TrafficRule
{
public:
    Destination(const TrafficRuleConfig& config,
                const std::shared_ptr<DependencyInjector>& injector);
    virtual ~Destination() = default;

    common::Status ApplyRule(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info);

private:
    int MakeDecisions(Frame* const frame,
                      ReferenceLineInfo* const reference_line_info);
};

}  // namespace planning
}  // namespace apollo
