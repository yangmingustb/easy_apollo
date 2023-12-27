/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once
#include "collision_request.h"
#include "collision_result2d.h"

namespace cdl
{
namespace bp2d
{
struct CollisionData
{
    CollisionData() : done(false) {}
    CollisionRequest request;
    CollisionResult result;

    /** Whether the collision iteration can stop */
    bool done;
};  // class CollisionData

}  // namespace bp2d
}  // namespace cdl