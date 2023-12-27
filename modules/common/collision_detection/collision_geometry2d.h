/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once
#include <memory>
#include "aabb2d.h"

namespace cdl
{
namespace bp2d
{
enum OBJECT_TYPE
{
    OT_UNKNOWN,
    OT_BVH,
    OT_GEOM,
    OT_COUNT
};

enum NODE_TYPE
{
    BV_UNKNOWN,
    BV_AABB,
    GEOM_BOX,
    GEOM_CONVEX,
    NODE_COUNT
};

/* The geometry for the object for collision or distance computation */
class CDL_EXPORT CollisionGeometry
{
public:
    CollisionGeometry();

    virtual ~CollisionGeometry();

    /** get the type of the object */
    virtual OBJECT_TYPE getObjectType() const;

    /** get the node type */
    virtual NODE_TYPE getNodeType() const;

    /** compute the AABB for object in local coordinate */
    virtual void computeLocalAABB() = 0;

    /** get user data in geometry */
    void *getUserData() const;

    /** set user data in geometry */
    void setUserData(void *data);

    /** whether the object is completely occupied */
    bool isOccupied() const;

    /** whether the object is completely free */
    bool isFree() const;

    /** whether the object has some uncertainty */
    bool isUncertain() const;

    /** AABB center in local coordinate */
    Vector2r aabb_center;

    /** AABB radius */
    real aabb_radius;

    /** AABB in local coordinate, used for tight AABB when only translation
     * transform */
    AABB aabb_local;

    /** pointer to user defined data specific to this object */
    void *user_data;

    /** collision cost for unit volume */
    real cost_density;

    /** threshold for occupied ( >= is occupied) */
    real threshold_occupied;

    /** threshold for free (<= is free) */
    real threshold_free;

    /** compute the volume */
    virtual real computeVolume() const;
};  // class CollisionGeometry
}  // namespace bp2d

}  // namespace cdl
