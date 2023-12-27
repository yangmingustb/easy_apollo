/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "collision_object2d.h"

namespace cdl
{
namespace bp2d
{
CollisionObject::CollisionObject(CollisionGeometry *cgeom_) : geom(cgeom_)
{
    geom->computeLocalAABB();
    computeAABB2();
}

void CollisionObject::setObject(CollisionGeometry *cgeom_)
{
    geom = cgeom_;
    geom->computeLocalAABB();
    computeAABB2();
}

CollisionObject::~CollisionObject() {}

OBJECT_TYPE CollisionObject::getObjectType() const
{
    return geom->getObjectType();
}

NODE_TYPE CollisionObject::getNodeType() const { return geom->getNodeType(); }

const AABB &CollisionObject::getAABB() const { return aabb; }

void CollisionObject::computeAABB2() { aabb = geom->aabb_local; }

void *CollisionObject::getUserData() const { return user_data; }

void CollisionObject::setUserData(void *data) { user_data = data; }

Vector2r CollisionObject::getAabbCenter() const { return (geom->aabb_center); }

real CollisionObject::getRadius() const { return geom->aabb_radius; }

CollisionGeometry *CollisionObject::collisionGeometry() const { return geom; }

bool CollisionObject::isOccupied() const { return geom->isOccupied(); }

bool CollisionObject::isFree() const { return geom->isFree(); }

bool CollisionObject::isUncertain() const { return geom->isUncertain(); }

}  // namespace bp2d
}  // namespace cdl
