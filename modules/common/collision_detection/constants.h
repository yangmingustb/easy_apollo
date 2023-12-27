/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once
#include "macro.h"
#include "types.h"

namespace cdl
{
/** convex hull vertex number */
static constexpr uint32_t MAX_CONVEX_VERTEX_NUM = 12;

/** max vehicle trajectory size */
static constexpr uint32_t MAX_VEH_TRAJ_SIZE = 300;

/** malloc a memory pool for the vehicle trajectory tree */
static constexpr uint32_t MAX_VEH_NODE_POOL_SIZE = 600;

/** max number of the collision pair in the struct CollisionResult */
static constexpr uint32_t MAX_COLLISION_NUM = 10000;

/** max leaf node number of the obstalce trajectory set tree */
static constexpr uint32_t MAX_LEAF_NODE_NUM = 5000;

/** malloc a memory pool for the obstacle trajectory set tree */
static constexpr uint32_t MAX_NODE_POOL_SIZE = 10000;

struct CDL_EXPORT constants
{
    static constexpr real pi() { return real(M_PI); }
    static constexpr real eps() { return real(1.e-7); }
    static constexpr real gjk_tolorance() { return real(1e-5); }
};

}  // namespace cdl
