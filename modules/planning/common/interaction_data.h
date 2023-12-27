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

#include "modules/common/math/box2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace apollo
{
namespace planning
{

// 不管是ego,还是其他agent，都有意图、交互类型,相对于ego的位置
enum class agent_relative_position
{
    FRONT_SIDE, 
    BACK_SIDE, 
    LEFT_SIDE, 
    RIGHT_SIDE, 
};

// estimated agent
enum class agent_motion_state
{
    UNKNOWN = 0,
    STOP = 1,
    MOVING = 3,
    LOW_ACCELERATION = 4,
    HIGH_ACCELERATION = 5,
    LOW_DECELERATION = 6,
    HIGH_DECELERATION = 7,
};

enum class lane_drive_intention
{
    UNKNOWN = 0,
    LANE_CHANGING = 1,
    LANE_BORROW_FORWARD_LANE = 2,
    LANE_BORROW_REVERSE_LANE = 3,
    // 车道保持，且offset保持不变
    LANE_FOLLOW_AND_OFFSET_KEEPING = 4,

    // 车道保持，且offset有变化
    LANE_FOLLOW_AND_OFFSET_JUMPING = 5,
};

// 相对运动方向
enum class agent_drive_direction
{
    UNKNOWN = 0,
    STOPPING = 1,
    FORWARD_DRIVING = 2,
    CROSS_DRIVING = 3,
    REVERSE_DRIVING = 4,
};

// 描述path位置关系
enum class agent_lat_interaction
{
    UNKOWN = 0,
    // path交叉
    CROSS = 2,
    // path merge,起点no overlap, but other point overlap
    MERGE = 3,
    // path 不存在overlap，且path在另一个path附近，就是side pass
    SIDE_PASS = 4,

    // 有一个path的起点已经重叠，一个path在另一个path的前方
    OVERLAP = 5,
};

// 描述st 关系，可以是ego相对于agent，也可以是agent 相对于ego
enum class agent_lon_interaction
{
    UNKNOWN = 0,
    STOP = 1,
    IGNORE = 3,
    FOLLOW = 4,
    YIELD = 5,
    OVERTAKE = 6,

    // 虽然path没有重叠，没有得到st
    // bound，但是也会产生纵向决策，比如两者path距离太小，为了安全起见，需要减速决策
    CAUTION = 7,
};

}  // namespace planning
}  // namespace apollo
