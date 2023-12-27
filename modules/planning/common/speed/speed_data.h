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

#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "cyber/common/log.h"

namespace apollo
{
namespace planning
{

enum class speed_mode
{
    hard_acceleration,
    soft_acceleration,
    constant_speed,
    soft_brake,
    hard_brake
};

// 规划结果，参考速度，都可以存储
class SpeedData : public std::vector<common::SpeedPoint>
{
public:
    SpeedData() = default;

    virtual ~SpeedData() = default;

    explicit SpeedData(std::vector<common::SpeedPoint> speed_points);

    void AppendSpeedPoint(const double s, const double time, const double v,
                          const double a, const double da);

    bool EvaluateByTime(const double time,
                        common::SpeedPoint* const speed_point) const;

    // Assuming spatial traversed distance is monotonous, which is the case for
    // current usage on city driving scenario
    bool EvaluateByS(const double s,
                     common::SpeedPoint* const speed_point) const;

    double TotalTime() const;

    // Assuming spatial traversed distance is monotonous
    double TotalLength() const;

    virtual std::string DebugString() const;

    int log_speed_data() const
    {
        for (size_t i = 0; i < size(); i++)
        {
            AINFO << "s " << at(i).s() << ", t " << at(i).t() << ", v "
                  << at(i).v() << ", a " << at(i).a() << ", jerk "
                  << at(i).da();
        }

        return 0;
    }

};

}  // namespace planning
}  // namespace apollo
