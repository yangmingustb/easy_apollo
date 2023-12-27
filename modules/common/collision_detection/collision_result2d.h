/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#include <set>
#include <vector>
#include "constants.h"
#include "contact2d.h"
#include "types.h"

namespace cdl
{
namespace bp2d
{
inline bool compareID(const Contact &ct1, const Contact &ct2)
{
    /** rank by b1, then by penetration_depth */
    return (ct1.b1 < ct2.b1) ||
           ((ct1.b1 == ct2.b1) && (ct1.distance < ct2.distance));
};

inline bool compareIDOnly(const Contact &ct1, const Contact &ct2)
{
    return (ct1.b1 < ct2.b1) || (ct1.b1 == ct2.b1);
};

inline bool compare2ID(const Contact &ct1, const Contact &ct2)
{
    /** rank by b1, and then b2 */
    return ((ct1.b1 < ct2.b1) || ((ct1.b1 == ct2.b1) && (ct1.b2 < ct2.b2)));
};

inline bool compareByFirstID(const Contact &ct1, const Contact &ct2)
{
    return (ct1.b1 < ct2.b1);
}

struct CDL_EXPORT CollisionResult
{
    /** contacts result for collision cases */
    Contact contacts[cdl::MAX_COLLISION_NUM];
    int32_t contact_size_;

    /** distance pair for separate cases */
    Contact distance_pairs[cdl::MAX_COLLISION_NUM];
    int32_t contact_pair_size_;

    Vector2r cached_gjk_guess;
    real min_distance{std::numeric_limits<real>::max()};

    /** Only record the aabb collision pairs in aabb tree, this value >=
     * contacts.size() */
    int32_t aabb_collision_pair;

    CollisionResult();

    /** add one contact into results */
    void addContact(const Contact &c);

    /** add one dist pair into results */
    void addDistancePair(const Contact &dist_pair);

    /** return binary collision result */
    bool isCollision() const;

    /** number of contacts  */
    int32_t numContacts() const;

    /** number of distance pairs */
    int32_t numDistancePairs() const;

    /** number of cost sources found */
    int32_t numCostSources() const;

    /** get the i-th contact calculated */
    Contact getContact(const int32_t i) const;

    /**
     * \brief Get the Contact object
     *
     * \param[out] c
     * \param[in] i
     */
    Contact *getContact(const int32_t i);

    /** get all the contacts */
    void getContacts(Contact *cts_, int32_t *size);

    /** get distance pair by index */
    Contact getDistancePair(const int32_t i) const;

    void getDistancePair(Contact *ct, const int32_t i) const;

    /** get distance pair vector */
    void getDistancePairs(Contact *dist_pairs_, int32_t *size);

    /** clear the results obtained */
    void clear();
};  // class CollisionResult
}  // namespace bp2d
}  // namespace cdl
