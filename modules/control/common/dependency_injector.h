/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo
{
namespace control
{
class DependencyInjector
{
public:
    DependencyInjector() = default;

    ~DependencyInjector() = default;

    apollo::common::VehicleStateProvider* vehicle_state()
    {
        return &vehicle_state_;
    }

private:
    //  这里的state是cyber中接收到的state,
    //  如果考虑到接收延时，以及计算延时，车辆已经不是当前的状态了
    // todo: 
    apollo::common::VehicleStateProvider vehicle_state_;
};

}  // namespace control
}  // namespace apollo
