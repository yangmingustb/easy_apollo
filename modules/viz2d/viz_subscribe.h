/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/pad_msg.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/storytelling/proto/story.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo
{
/**
 * @struct local_view
 * @brief LocalView contains all necessary data as viz
 * * input,每一帧的数据,有的数据是每一帧刷新，有的数据是历史数据，注意：cyber
 *   rt传输的数据最好拷贝到这里
 */

struct viz_subscribe
{
    prediction::PredictionObstacles prediction_obstacles;
    
    canbus::Chassis chassis;
    
    localization::LocalizationEstimate localization_estimate;
    
    perception::TrafficLightDetection traffic_light;

    routing::RoutingResponse routing;
    // relative_map::MapMsg relative_map;
    // storytelling::Stories stories;
    control::ControlCommand control;

    // perception publish obs. not hmi generated obs
    perception::PerceptionObstacles perception;

    planning::ADCTrajectory traj;

};

}