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
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "modules/planning/common/frame.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/task_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"
#include "modules/planning/tasks/deciders/st_bounds_decider/st_driving_limits.h"
#include "modules/planning/tasks/deciders/st_bounds_decider/st_guide_line.h"
#include "modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

namespace apollo
{
namespace planning
{
constexpr double kSTBoundsDeciderResolution = 0.1;
constexpr double kSTPassableThreshold = 3.0;

// 生成obs的st bound
// 进行st decision？
// st空间是non convex, 如何确定在哪一个方向通行：
// 这里都是基于规则的搜索s
// boundary，这种方式在简单环境中可行，但是在复杂环境中有存在无解。
// 所以在环境复杂的时候，不论是path还是speed，DP搜索都比基于规则的判断要好。

// 这是速度规划最复杂的模块，可以参考：https://zhuanlan.zhihu.com/p/574219017

// 这个类，会做部分决策，还是全部决策，最后生成st boundary

// lon decision 
//     ObjectIgnore ignore = 1;
//     ObjectStop stop = 2;
//     ObjectFollow follow = 3;
//     ObjectYield yield = 4;
//     ObjectOvertake overtake = 5;

// if static: stop

struct st_bound_point
{
    // STBoundPoint contains (t, s_min, s_max)

    double t;
    double s_min;
    double s_max;
};

struct st_object_decision
{
    // std::string, ObjectDecisionType
    std::string id;
    ObjectDecisionType type;
};

struct st_obs_decision_set
{
    // ObsDecSet is a set of decision for new obstacles.
    // using ObsDecSet = std::vector<std::pair<std::string,
    // ObjectDecisionType>>;

    std::vector<st_object_decision> obs_decision_set_;
};

class STBoundsDecider : public Decider
{
public:
    STBoundsDecider(const TaskConfig& config,
                    const std::shared_ptr<DependencyInjector>& injector);

private:
    common::Status Process(
            Frame* const frame,
            ReferenceLineInfo* const reference_line_info) override;

    void InitSTBoundsDecider(const Frame& frame,
                             ReferenceLineInfo* const reference_line_info);

    // no call
    common::Status GenerateFallbackSTBound(
            std::vector<std::tuple<double, double, double>>* const st_bound,
            std::vector<std::tuple<double, double, double>>* const vt_bound);

    common::Status GenerateRegularSTBound(
            std::vector<std::tuple<double, double, double>>* const st_bound,
            std::vector<std::tuple<double, double, double>>* const vt_bound,
            std::vector<std::pair<double, double>>* const st_guide_line);

    void RemoveInvalidDecisions(
            std::pair<double, double> driving_limit,
            std::vector<std::pair<
                    std::tuple<double, double, double>,
                    std::vector<std::pair<std::string, ObjectDecisionType>>>>*
                    available_choices);

    void RankDecisions(
            double s_guide_line, std::pair<double, double> driving_limit,
            std::vector<std::pair<
                    std::tuple<double, double, double>,
                    std::vector<std::pair<std::string, ObjectDecisionType>>>>*
                    available_choices);

    bool BackwardFlatten(
            std::vector<std::tuple<double, double, double>>* const st_bound);

    void RecordSTGraphDebug(
            const std::vector<STBoundary>& st_graph_data,
            const std::vector<std::tuple<double, double, double>>& st_bound,
            const std::vector<std::pair<double, double>>& st_guide_line,
            planning_internal::STGraphDebug* const st_graph_debug);

private:
    STBoundsDeciderConfig st_bounds_config_;

    // 匀速，模型
    STGuideLine st_guide_line_;
    STDrivingLimits st_driving_limits_;
    STObstaclesProcessor st_obstacles_processor_;
};

}  // namespace planning
}  // namespace apollo
