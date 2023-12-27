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
 * @file speed_limit.h
 **/

#pragma once

#include <utility>
#include <vector>
#include "cyber/common/log.h"

namespace apollo
{
namespace planning
{

struct speed_point_limit
{
    double v_upper;
    double v_lower;

    double acc_upper;
    double acc_lower;

    double jerk_upper;
    double jerk_lower;
};

// 限速从哪儿来：
// 弯道限速怎么生成,使用曲率生成.
class SpeedLimit
{
public:
    SpeedLimit() = default;

    void AppendSpeedLimit(const double s, const double v);

    const std::vector<std::pair<double, double>>& speed_limit_points() const;

    // 这个s是ref line s，还是path s
    double GetSpeedLimitByS(const double s) const;

    void set_speed_limit_by_index(int index, const double v)
    {
        if (index < 0 || index >= speed_limit_points_.size())
        {
            return;
        }

        speed_limit_points_[index].second = v;
    }

    void Clear();

    void debug_string()
    {

        for (size_t i = 0; i < speed_limit_points_.size(); i++)
        {
            AINFO << "s: " << speed_limit_points_[i].first << ", v "
                  << speed_limit_points_[i].second;
        }

        return;
    }

    int get_index_by_s(double s);

private:
    // use a vector to represent speed limit
    // the first number is s, the second number is v
    // It means at distance s from the start point, the speed limit is v.
    // 这里s都是path s
    std::vector<std::pair<double, double>> speed_limit_points_;
};

}  // namespace planning
}  // namespace apollo
