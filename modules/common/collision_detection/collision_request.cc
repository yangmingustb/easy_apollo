/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "collision_request.h"

namespace cdl
{
namespace bp2d
{
CollisionRequest::CollisionRequest()
    : num_max_contacts(1),
      enable_contact(false),
      enable_distance_request(false),
      distance_lower_bound(10.0),
      enable_cached_gjk_guess(false),
      enable_filter(false),
      filter_set(),
      cached_gjk_guess(Vector3r::UnitX()),
      gjk_tolerance(1e-6)
{
}

bool CollisionRequest::isSatisfied(const CollisionResult &result) const
{
    return result.isCollision() && (num_max_contacts <= result.numContacts());
}

}  // namespace bp2d
}  // namespace cdl
