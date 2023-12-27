/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once
#include "cdl_math.h"
#include "types.h"

namespace cdl
{
namespace bp2d
{
class CDL_EXPORT AABB
{
public:
    /** The min point in the AABB */
    Vector2r min_;

    /** The max point in the AABB */
    Vector2r max_;

    /** Creating an AABB with zero size (low bound +inf, upper bound -inf)*/
    AABB();

    /** Creating an AABB at position center with zero size */
    AABB(const Vector2r &center);

    /** Creating an AABB with two endpoints a and b */
    AABB(const Vector2r &a, const Vector2r &b);

    /** Creating an AABB centered as core and is of half-dimension delta */
    AABB(const AABB &core, const Vector2r &delta);

    /** Creating an AABB contains three points */
    AABB(const Vector2r &a, const Vector2r &b, const Vector2r &c);

    /** Check whether two AABB are overlap */
    bool overlap(const AABB &other) const;

    /** Check whether the AABB contains another AABB */
    bool contain(const AABB &other) const;

    /** Check whether two AABB are overlapped along specific axis */
    bool axisOverlap(const AABB &other, int axis_id) const;

    /** Check whether two AABB are overlap and return the overlap part */
    bool overlap(const AABB &other, AABB &overlap_part) const;

    /** Check whether the AABB contains a point */
    bool contain(const Vector2r &p) const;

    void combine(const AABB &a, const AABB &b)
    {
        cdl_min(&min_, a.min_, b.min_);
        cdl_max(&max_, a.max_, b.max_);
    }

    void combine(const AABB &a)
    {
        min_(0) = std::min(min_(0), a.min_(0));
        min_(1) = std::min(min_(1), a.min_(1));
        max_(0) = std::max(max_(0), a.max_(0));
        max_(1) = std::max(max_(1), a.max_(1));
    }

    /** Merge the AABB and a point */
    AABB &operator+=(const Vector2r &p);

    /** Merge the AABB and another AABB */
    AABB &operator+=(const AABB &other);

    /** Return the merged AABB of current AABB and the other one */
    AABB operator+(const AABB &other) const;

    /** Width of the AABB */
    real width() const;

    /** Height of the AABB */
    real height() const;

    /** Area of the AABB */
    real area() const;

    /** Perimeter of the AABB */
    real perimeter() const;

    /** Size of the AABB (used in BV_Splitter to order two AABB2Ds) */
    real size() const;

    /** Radius of the AABB */
    real radius() const;

    /** Center of the AABB */
    Vector2r center() const;

    /**
     * \brief  Distance between two AABB2Ds;
     * \param[in]  other   : the second aabb2D
     * \param[out]  P       : the witness point
     * \param[out]  Q       : the witness point on the other aabb2D
     * \return the distance
     */
    real distance(const AABB &other, Vector2r *P, Vector2r *Q) const;

    /** Distance between two AABB2Ds */
    real distance(const AABB &other) const;

    /** whether two AABB are equal */
    bool equal(const AABB &other) const;

    /** expand the half size of the AABB by delta, and keep the center
     *  unchanged. */
    AABB &expand(const Vector2r &delta);

    /** expand the aabb2D by increase the thickness of the plate by a
     * ratio */
    AABB &expand(const AABB &core, real ratio);

    static AABB translate(const AABB &aabb, const Vector2r &t);
};  // class AABB

}  // namespace bp2d
}  // namespace cdl
