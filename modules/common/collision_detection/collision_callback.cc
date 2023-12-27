/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "collision_callback.h"
#include "epa2d.h"
#include "gjk2d.h"

namespace cdl
{
namespace bp2d
{
static cdl::np2d::ShapeProxy2D bodyA, bodyB;
static cdl::np2d::GJK2D gjk_solver;
static cdl::np2d::DistResult2D gjk_result;
static cdl::np2d::EPA2D epa_solver;
static Contact ct;
static cdl::UserData *ud1;
static cdl::UserData *ud2;

std::size_t collide(const CollisionObject *o1, const CollisionObject *o2,
                    CollisionResult &result)
{
    if (o1->collisionGeometry()->getNodeType() == GEOM_CONVEX)
    {
        const Convex *obj1 =
                static_cast<const Convex *>(o1->collisionGeometry());
        obj1->getBoundVertices(&bodyA);
    }
    else if (o1->collisionGeometry()->getNodeType() == GEOM_BOX)
    {
        const Box *obj1 = static_cast<const Box *>(o1->collisionGeometry());
        obj1->getBoundVertices(&bodyA);
    }

    if (o2->collisionGeometry()->getNodeType() == GEOM_CONVEX)
    {
        const Convex *obj2 =
                static_cast<const Convex *>(o2->collisionGeometry());
        obj2->getBoundVertices(&bodyB);
    }
    else if (o2->collisionGeometry()->getNodeType() == GEOM_BOX)
    {
        const Box *obj2 = static_cast<const Box *>(o2->collisionGeometry());
        obj2->getBoundVertices(&bodyB);
    }

    bool collision = gjk_solver.Collision(bodyA, bodyB);

#if 0
    printf("judge \n");
    if (!collision)
    {
        printf("no collision \n");
    }
    else
    {
        printf("collision \n");
    }
#endif

    if (collision)
    {
        ct.clear();
        ud1 = static_cast<cdl::UserData *>(o1->getUserData());
        ud2 = static_cast<cdl::UserData *>(o2->getUserData());

        if (ud1 != nullptr)
        {
            ct.b1 = ud1->id;
        }
        else
        {
            printf("collision callback ud 1 nullptr.\n");
        }
        if (ud2 != nullptr)
        {
            ct.b2 = ud2->id;
        }
        else
        {
            printf("collision callback ud 2 nullptr.\n");
        }

        result.addContact(ct);
        return result.numContacts();
    }
    else
    {
        return 0;
    }
}

bool CollisionFunction(CollisionObject *o1, CollisionObject *o2,
                       CollisionData *cdata)
{
    const auto &request = cdata->request;
    auto &result = cdata->result;

    if (cdata->done) return true;

    collide(o1, o2, result);

    if (result.isCollision() &&
        result.numContacts() >= request.num_max_contacts)
    {
        cdata->done = true;
    }

    return cdata->done;
}

real distance(const CollisionObject *o1, const CollisionObject *o2,
              const CollisionRequest &request, CollisionResult &result)
{
    cdl::real distance;

    ct.distance = std::numeric_limits<cdl::real>::infinity();
    ct.b1 = -1;
    ct.b2 = -1;
    if (o1->collisionGeometry()->getNodeType() == GEOM_CONVEX)
    {
        const Convex *obj1 =
                static_cast<const Convex *>(o1->collisionGeometry());
        obj1->getBoundVertices(&bodyA);
    }
    else if (o1->collisionGeometry()->getNodeType() == GEOM_BOX)
    {
        const Box *obj1 = static_cast<const Box *>(o1->collisionGeometry());
        obj1->getBoundVertices(&bodyA);
    }

    if (o2->collisionGeometry()->getNodeType() == GEOM_CONVEX)
    {
        const Convex *obj2 =
                static_cast<const Convex *>(o2->collisionGeometry());
        obj2->getBoundVertices(&bodyB);
    }
    else if (o2->collisionGeometry()->getNodeType() == GEOM_BOX)
    {
        const Box *obj2 = static_cast<const Box *>(o2->collisionGeometry());
        obj2->getBoundVertices(&bodyB);
    }

    gjk_solver.Distance(bodyA, bodyB, &gjk_result);

    if (request.enable_signed_distance_request)
    {
        if (gjk_result.status == cdl::np2d::CollisionStatus::PENETRATION)
        {
            epa_solver.Penetration(bodyA, bodyB, gjk_solver.simplex_,
                                   gjk_result);
        }
    }
    distance = gjk_result.distance;
    result.min_distance = request.distance_lower_bound;
    if (distance <= cdl::constants::eps())
    {
        ud1 = static_cast<cdl::UserData *>(o1->getUserData());
        ud2 = static_cast<cdl::UserData *>(o2->getUserData());
        ct.b1 = ud1->id;
        ct.b2 = ud2->id;
        cdl::Vector2r p, q;
        p = gjk_result.p;
        q = gjk_result.q;

        ct.pos = p;
        ct.normal = q - p;
        ct.distance = distance;
        result.addContact(ct);
    }
    else if (distance < request.distance_lower_bound &&
             distance > cdl::constants::eps())
    {
        ud1 = static_cast<cdl::UserData *>(o1->getUserData());
        ud2 = static_cast<cdl::UserData *>(o2->getUserData());
        ct.b1 = ud1->id;
        ct.b2 = ud2->id;
        cdl::Vector2r p, q;
        p = gjk_result.p;
        q = gjk_result.q;
        ct.pos = p;
        ct.normal = q - p;
        ct.distance = distance;
        result.addDistancePair(ct);
    }
    return distance;
}

bool DistanceFunction(CollisionObject *o1, CollisionObject *o2,
                      CollisionData *cdata, real &min_distance)
{
    const auto &request = cdata->request;
    auto &result = cdata->result;

    if (cdata->done)
    {
        min_distance = result.min_distance;
        return true;
    }

    distance(o1, o2, request, result);

    min_distance = result.min_distance;

    if (min_distance <= cdl::constants::eps() &&
        result.numContacts() >= request.num_max_contacts)
    {
        cdata->done = true;
        return true;
    }

    return cdata->done;
}

}  // namespace bp2d
}  // namespace cdl
