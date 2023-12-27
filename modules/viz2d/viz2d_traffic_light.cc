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

#include "viz2d_traffic_light.h"

namespace apollo
{

using apollo::common::color::ANSI_GREEN;
using apollo::common::color::ANSI_RED;
using apollo::common::color::ANSI_RESET;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::SignalInfoConstPtr;
using apollo::localization::LocalizationEstimate;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;

DEFINE_bool(all_lights, false, "set all lights on the map");

DEFINE_double(traffic_light_distance, 1000.0,
              "only retrieves traffic lights within this distance");

bool ManualTrafficLight::Init()
{
    has_localization_ = false;
    green_light_start_time_ = -1.0;
    return true;
}

bool ManualTrafficLight::Proc(
        const LocalizationEstimate &localization,
        perception::TrafficLightDetection *traffic_light_detection,
        perception::TrafficLight::Color color)
{
    localization_.CopyFrom(localization);

    if (localization_.has_header())
    {
        has_localization_ = true;
    }
    else
    {
        return false;
    }

    traffic_light_detection->clear_traffic_light();

    std::vector<SignalInfoConstPtr> signals;

    bool result = false;
    if (FLAGS_all_lights)
    {
        result = GetAllTrafficLights(&signals);
    }
    else
    {
        result = GetTrafficLightsWithinDistance(&signals);
    }

    if (!result)
    {
        AERROR << "Failed to get traffic signals from current location on "
                  "map";
    }
    else
    {
        AINFO << "Received " << signals.size() << " traffic lights";
    }

    ADEBUG << "Color: " << TrafficLight::Color_Name(color);

    CreateTrafficLightDetection(signals, color, traffic_light_detection);

    green_light_over_ =
            green_light_change_to_red_light(traffic_light_detection, color);

    return true;
}

bool ManualTrafficLight::GetAllTrafficLights(
        std::vector<SignalInfoConstPtr> *traffic_lights)
{
    static auto map_filename = apollo::hdmap::BaseMapFile();
    static apollo::hdmap::Map map_proto;
    static std::vector<SignalInfoConstPtr> map_traffic_lights;

    if (map_proto.lane().empty() && map_traffic_lights.empty())
    {
        AERROR << "signal size: " << map_proto.signal_size();
        if (absl::EndsWith(map_filename, ".xml"))
        {
            if (!apollo::hdmap::adapter::OpendriveAdapter::LoadData(
                        map_filename, &map_proto))
            {
                return false;
            }
        }
        else if (!apollo::cyber::common::GetProtoFromFile(map_filename,
                                                          &map_proto))
        {
            return false;
        }
        for (const auto &signal : map_proto.signal())
        {
            const auto *hdmap = HDMapUtil::BaseMapPtr();
            if (!hdmap)
            {
                AERROR << "Invalid HD Map.";
                return false;
            }
            map_traffic_lights.push_back(hdmap->GetSignalById(signal.id()));
        }
    }

    *traffic_lights = map_traffic_lights;

    return true;
}

bool ManualTrafficLight::GetTrafficLightsWithinDistance(
        std::vector<SignalInfoConstPtr> *traffic_lights)
{
    CHECK_NOTNULL(traffic_lights);

    if (!has_localization_)
    {
        AERROR << "No localization received";
        return false;
    }

    const auto *hdmap = HDMapUtil::BaseMapPtr();
    if (!hdmap)
    {
        AERROR << "Invalid HD Map.";
        return false;
    }

    auto position = localization_.pose().position();
    int ret = hdmap->GetForwardNearestSignalsOnLane(
            position, FLAGS_traffic_light_distance, traffic_lights);

    if (ret != 0)
    {
        AERROR << "failed to get signals from position: "
               << position.ShortDebugString()
               << " with distance: " << FLAGS_traffic_light_distance
               << " on map";

        return false;
    }

    return true;
}

bool ManualTrafficLight::CreateTrafficLightDetection(
        const std::vector<SignalInfoConstPtr> &signals,
        TrafficLight::Color color, TrafficLightDetection *detection)
{
    CHECK_NOTNULL(detection);

    for (const auto &iter : signals)
    {
        auto *light = detection->add_traffic_light();
        light->set_color(color);
        light->set_confidence(1.0);
        light->set_tracking_time(1.0);
        light->set_id(iter->id().id());

#if debug_viz2d_traffic_light
        AINFO << iter->signal().DebugString();
#endif
    }

    auto *header = detection->mutable_header();
    double timestamp = ::apollo::cyber::Clock::NowInSeconds();
    header->set_module_name(FLAGS_traffic_light_detection_topic);
    header->set_timestamp_sec(timestamp);

    return true;
}

}  // namespace apollo
