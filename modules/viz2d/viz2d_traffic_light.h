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

#include <poll.h>

#include "absl/strings/match.h"
#include "absl/strings/str_join.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/color.h"
#include "modules/common/util/message_util.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"

namespace apollo
{

#define debug_viz2d_traffic_light (0)
#define green_light_over_time (100.0)

class ManualTrafficLight
{
public:
    bool Init();

    bool Proc(const localization::LocalizationEstimate &localization,
              perception::TrafficLightDetection *traffic_light_detection,
              perception::TrafficLight::Color color);

    bool green_light_change_to_red_light(
            perception::TrafficLightDetection *traffic_light_detection,
            perception::TrafficLight::Color color)
    {
        if (traffic_light_detection->has_header() &&
            color == perception::TrafficLight::GREEN)
        {
            // 绿灯开始时间
            if (green_light_start_time_ < 0.0)
            {
                green_light_start_time_ =
                        traffic_light_detection->header().timestamp_sec();
            }
            else
            {
                double latest_time =
                        traffic_light_detection->header().timestamp_sec();

                double delta_time = latest_time - green_light_start_time_;
                if (delta_time > green_light_over_time)
                {
                    return true;
                }
            }
        }
        else
        {
            green_light_start_time_ = -1.0;
        }

        return false;
    }

    bool is_green_light_over()
    {
        return green_light_over_;
    }

private:
    bool GetAllTrafficLights(
            std::vector<hdmap::SignalInfoConstPtr> *traffic_lights);

    bool GetTrafficLightsWithinDistance(
            std::vector<hdmap::SignalInfoConstPtr> *traffic_lights);

    bool CreateTrafficLightDetection(
            const std::vector<hdmap::SignalInfoConstPtr> &signals,
            perception::TrafficLight::Color color,
            perception::TrafficLightDetection *detection);

private:
    localization::LocalizationEstimate localization_;

    bool has_localization_;

    // 如果交通灯是绿色，那么30秒之后，自动变成red light
    double green_light_start_time_;

    bool green_light_over_;
};

}  // namespace apollo
