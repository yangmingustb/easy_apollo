/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once
#include <memory>
#include "collision_geometry2d.h"

namespace cdl
{
namespace bp2d
{
/**
 * \brief the object for collision or distance computation, contains the
 * geometry and the transform information
 */
class CDL_EXPORT CollisionObject
{
public:
    CollisionObject() : geom(nullptr), user_data(nullptr){};

    CollisionObject(CollisionGeometry *cgeom);

    ~CollisionObject();

    void setObject(CollisionGeometry *cgeom);

    /** get the type of the object */
    OBJECT_TYPE getObjectType() const;

    /** get the node type */
    NODE_TYPE getNodeType() const;

    /** get the AABB in world space */
    const AABB &getAABB() const;

    /** compute the AABB in world space, implement for const
     * geometry, no pointer form */
    void computeAABB2();

    /** get user data in object */
    void *getUserData() const;

    /** set user data in object */
    void setUserData(void *data);

    Vector2r getAabbCenter() const;

    real getRadius() const;

    /** get geometry from the object instance */
    CollisionGeometry *collisionGeometry() const;

    /** whether the object is completely occupied */
    bool isOccupied() const;

    /** whether the object is completely free */
    bool isFree() const;

    /** whether the object is uncertain */
    bool isUncertain() const;

protected:
    CollisionGeometry *geom;

    /** AABB in global coordinate */
    mutable AABB aabb;

    /** pointer to user defined data specific to this object */
    void *user_data;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};  // class CollisionObject
}  // namespace bp2d
}  // namespace cdl
