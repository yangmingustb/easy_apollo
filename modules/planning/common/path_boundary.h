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

#include <string>
#include <utility>
#include <vector>

#include "modules/planning/common/path/path_data.h"

namespace apollo
{
namespace planning
{



class PathBoundary
{
public:
    PathBoundary(const double start_s, const double delta_s,
                 std::vector<std::pair<double, double>> path_boundary);

    virtual ~PathBoundary() = default;

    double start_s() const;

    double delta_s() const;

    void set_boundary(const std::vector<std::pair<double, double>>& boundary);
    const std::vector<std::pair<double, double>>& boundary() const;

    void set_label(const std::string& label);
    const std::string& label() const;

    void set_blocking_obstacle_id(const std::string& obs_id);
    const std::string& blocking_obstacle_id() const;

    void set_target_offset(double x) { target_offset_ = x; }

    double get_target_offset () const
    {
        return   target_offset_;
    }

private:
    // 起点在ref line的投影s
    double start_s_ = 0.0;
    double delta_s_ = 0.0;

    // lower, upper
    std::vector<std::pair<double, double>> boundary_;
    std::string label_ = "regular";

    // fallback path boundary 没有障碍物信息
    std::string blocking_obstacle_id_ = "";


    // human lane borrow时会赋值，未来会retired
    double target_offset_ = 0.0;

public:
    path_boundary_type type_;
};

}  // namespace planning
}  // namespace apollo
