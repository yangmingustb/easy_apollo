/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once
#include "box2d.h"
#include "collision_data2d.h"
#include "collision_object2d.h"
#include "collision_request.h"
#include "constants.h"
#include "contact2d.h"
#include "convex2d.h"
#include "user_data.h"

namespace cdl
{
namespace bp2d
{
/**
 * \brief Main collision interface: given two collision objects, and the
 * requirements for contacts, including num of max contacts, this function
 * performs the collision detection between them.
 * \param  o1            : collisoin object 1
 * \param  o2            : collisoin object 2
 * \param  result        : the collision detection results
 * \return The number of contacts
 */
std::size_t collide(const CollisionObject *o1, const CollisionObject *o2,
                    CollisionResult &result);
/**
 * \brief The complete collision callback function for broadphase and
 * narrowphase.
 *
 * \param o1            : collision object 1
 * \param o2            : collision object 2
 * \param cdata         : collision data
 * \return true         : if collision data is done
 * \return false        : if collision data is not done
 */
bool CollisionFunction(CollisionObject *o1, CollisionObject *o2,
                       CollisionData *cdata);

/**
 * \brief narrowphase collision detection wrapper for GJK algorithms
 * \param  o1            : collisoin object 1
 * \param  o2            : collision object 2
 * \param  request       : collision detection request, whether to do filtering,
 *                         the max number of contacts pairs
 * \param  result        : collision result filled by the algorithm
 * \return the distance
 */
real distance(const CollisionObject *o1, const CollisionObject *o2,
              const CollisionRequest &request, CollisionResult &result);

/**
 * \brief The complete distance callback function, all the distance pair
 * within the min distance threshold will be filled into the cdata
 * \param  o1            : collisoin object 1
 * \param  o2            : collision object 2
 * \param  cdata         : the data of collision detection request and results
 * \param  min_distance  : the distance threshold for the distance pairs
 * \return whether the collision detection is done
 */
bool DistanceFunction(CollisionObject *o1, CollisionObject *o2,
                      CollisionData *cdata, real &min_distance);

}  // namespace bp2d
}  // namespace cdl