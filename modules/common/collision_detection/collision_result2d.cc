/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "collision_result2d.h"

namespace cdl
{
namespace bp2d
{
CollisionResult::CollisionResult()
    : contact_size_(0), contact_pair_size_(0), aabb_collision_pair(0)
{
}

void CollisionResult::addContact(const Contact &c)
{
    contacts[contact_size_].set(c);
    contact_size_++;
}

void CollisionResult::addDistancePair(const Contact &distance_pair)
{
    distance_pairs[contact_pair_size_].set(distance_pair);
    contact_pair_size_++;
}

bool CollisionResult::isCollision() const { return contact_size_ > 0; }

int32_t CollisionResult::numContacts() const { return contact_size_; }

int32_t CollisionResult::numDistancePairs() const { return contact_pair_size_; }

Contact CollisionResult::getContact(const int32_t i) const
{
    return contacts[i];
}

Contact *CollisionResult::getContact(const int32_t i) { return contacts + i; }

Contact CollisionResult::getDistancePair(const int32_t i) const
{
    return distance_pairs[i];
}

void CollisionResult::getDistancePair(Contact *ct, const int32_t i) const
{
    ct->set(distance_pairs[i]);
}

void CollisionResult::getDistancePairs(Contact *dist_pairs_, int32_t *size)
{
    *size = contact_pair_size_;
    for (int32_t i = 0; i < contact_pair_size_; i++)
    {
        dist_pairs_[i].set(distance_pairs[i]);
    }
}

void CollisionResult::getContacts(Contact *cts_, int32_t *size)
{
    *size = contact_size_;
    for (int32_t i = 0; i < contact_size_; i++)
    {
        cts_[i].set(contacts[i]);
    }
}

void CollisionResult::clear()
{
    contact_size_ = 0;
    contact_pair_size_ = 0;
    aabb_collision_pair = 0;
}
}  // namespace bp2d
}  // namespace cdl
