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



#include "modules/planning/common/sl_boundary.h"

namespace apollo
{
namespace planning
{

bool is_lateral_overlap_for_sl_box(const SLBoundary *box0,
                                   const SLBoundary *box1)
{
    if (box0 == NULL || box1 == NULL)
    {
        return false;
    }

    if ((box0->start_l() > box1->end_l()))
    {
        return false;
    }
    else if ((box1->start_l() > box0->end_l()))
    {
        return false;
    }

    return true;
}

bool is_lateral_overlap_for_sl_box(const SLBoundary &box0,
                                   const SLBoundary &box1)
{

    if ((box0.start_l() > box1.end_l()))
    {
        return false;
    }
    else if ((box1.start_l() > box0.end_l()))
    {
        return false;
    }

    return true;
}

}  // namespace planning
}  // namespace apollo
