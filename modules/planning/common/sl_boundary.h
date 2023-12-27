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


#include "modules/planning/proto/sl_boundary.pb.h"

// 头文件包含时尽量精简，能够减少编译速度
namespace apollo
{
namespace planning
{

bool is_lateral_overlap_for_sl_box(const SLBoundary *box0,
                                   const SLBoundary *box1);

bool is_lateral_overlap_for_sl_box(const SLBoundary &box0,
                                   const SLBoundary &box1);

}  // namespace planning
}  // namespace apollo
