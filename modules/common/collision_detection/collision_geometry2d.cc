/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "collision_geometry2d.h"

namespace cdl
{
namespace bp2d
{
CollisionGeometry::CollisionGeometry()
    : aabb_center(Vector2r::Zero()),
      aabb_radius((real)0),
      user_data(nullptr),
      cost_density((real)1),
      threshold_occupied((real)1),
      threshold_free((real)0)
{
}

CollisionGeometry::~CollisionGeometry() {}

OBJECT_TYPE CollisionGeometry::getObjectType() const { return OT_UNKNOWN; }
NODE_TYPE CollisionGeometry::getNodeType() const { return BV_UNKNOWN; }
void *CollisionGeometry::getUserData() const { return user_data; }
void CollisionGeometry::setUserData(void *data) { user_data = data; }
bool CollisionGeometry::isOccupied() const
{
    return cost_density >= threshold_occupied;
}

bool CollisionGeometry::isFree() const
{
    return cost_density <= threshold_free;
}

bool CollisionGeometry::isUncertain() const
{
    return !isOccupied() && !isFree();
}

real CollisionGeometry::computeVolume() const { return 0; }

}  // namespace bp2d
}  // namespace cdl
