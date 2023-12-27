/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once
#include <unordered_set>
#include "collision_result2d.h"
#include "types.h"

namespace cdl
{
namespace bp2d
{
/**
 * \brief Parameters for performing collision request.
 */
struct CDL_EXPORT CollisionRequest
{
    /** The maximum number of contacts that can be returned. */
    int32_t num_max_contacts;

    /** If true, contact information will be returned. */
    bool enable_contact{false};

    /** whether to do distance query, including positice distance */
    bool enable_distance_request{false};

    /** positive and negative distance query */
    bool enable_signed_distance_request{false};

    /** distance threshould to do distance calculation for narrow phase */
    real distance_lower_bound{10.0};

    /** whether to cache the gjk simplex */
    bool enable_cached_gjk_guess{false};

    /** whether to enable filter */
    bool enable_filter{false};

    /** only do narrowphase collision or distance query with trajectory id in
     * this set */
    std::unordered_set<int32_t> filter_set;

    /** The initial guess to use in the GJK algorithm. */
    Vector3r cached_gjk_guess;

    real gjk_tolerance{1e-6};

    CollisionRequest();

    bool isSatisfied(const CollisionResult &result) const;
};

}  // namespace bp2d
}  // namespace cdl
