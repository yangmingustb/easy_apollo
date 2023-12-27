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
 * @file discretized_path.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "cyber/common/log.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo
{
namespace planning
{
// s是相对于轨迹起点的
// 如果想生成密集的path point，这里有线性插值方法
class DiscretizedPath : public std::vector<common::PathPoint>
{
public:
    DiscretizedPath() = default;

    explicit DiscretizedPath(std::vector<common::PathPoint> path_points);

    // 轨迹长度
    double Length() const;

    // 最后点的s值，相对于plan start
    double max_s() const;

    common::PathPoint Evaluate(const double path_s) const;

    common::PathPoint EvaluateReverse(const double path_s) const;

    int debug_string() const
    {
        AINFO << "max s: " << max_s();

        for (size_t i = 0; i < size(); i++)
        {
            const common::PathPoint &point = at(i);
            AINFO << point.DebugString();
        }

        return 0;
    }

    void set_path_boundary_type(path_boundary_type type) { type_ = type; }

    path_boundary_type get_path_boundary_type() const { return type_; }

    std::string get_path_boundary_name() const
    {
        switch (type_)
        {
            case path_boundary_type::PATH_BOUND_LANE_KEEP:
                return "lane_keep";
                break;
            case path_boundary_type::PATH_BOUND_LANE_BORROW_LEFT:
                return "lane_borrow_left";
                break;
            case path_boundary_type::PATH_BOUND_LANE_BORROW_RIGHT:
                return "lane_borrow_right";
                break;
            case path_boundary_type::PATH_BOUND_PULL_OVER:
                return "pull_over";
                break;
            case path_boundary_type::PATH_BOUND_LANE_CHANGE_LEFT:
                return "lane_change_left";
                break;
            case path_boundary_type::PATH_BOUND_LANE_CHANGE_RIGHT:
                return "lane_change_right";
                break;
            case path_boundary_type::PATH_BOUND_FALLBACK:
                return "fallback";
                break;

            default:
                break;
        }

        return "null";
    }

protected:
    std::vector<common::PathPoint>::const_iterator QueryLowerBound(
            const double path_s) const;
    std::vector<common::PathPoint>::const_iterator QueryUpperBound(
            const double path_s) const;

    path_boundary_type type_;
};

}  // namespace planning
}  // namespace apollo
