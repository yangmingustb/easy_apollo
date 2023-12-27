/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "aabb2d.h"
#include "gjk2d.h"

namespace cdl
{
namespace bp2d
{
AABB::AABB() : min_(Vector2r::Zero()), max_(Vector2r::Zero()) {}

AABB::AABB(const Vector2r &center) : min_(center), max_(center) {}

AABB::AABB(const Vector2r &a, const Vector2r &b)
    : min_(a.cwiseMin(b)), max_(a.cwiseMax(b))
{
}

AABB::AABB(const AABB &core, const Vector2r &delta)
    : min_(core.min_ - delta), max_(core.max_ + delta)
{
}

AABB::AABB(const Vector2r &a, const Vector2r &b, const Vector2r &c)
    : min_(a.cwiseMin(b).cwiseMin(c)), max_(a.cwiseMax(b).cwiseMax(c))
{
}

bool AABB::overlap(const AABB &other) const
{
    if ((min_.array() > other.max_.array()).any()) return false;
    if ((max_.array() < other.min_.array()).any()) return false;
    return true;
}

bool AABB::contain(const AABB &other) const
{
    if ((min_.array() > other.min_.array()).any()) return false;
    if ((max_.array() < other.max_.array()).any()) return false;
    return true;
}

bool AABB::axisOverlap(const AABB &other, int axis_id) const
{
    if (min_[axis_id] > other.max_[axis_id]) return false;
    if (max_[axis_id] < other.min_[axis_id]) return false;
    return true;
}

bool AABB::overlap(const AABB &other, AABB &overlap_part) const
{
    if (!overlap(other))
    {
        return false;
    }
    overlap_part.min_ = min_.cwiseMax(other.min_);
    overlap_part.max_ = max_.cwiseMin(other.max_);
    return true;
}

bool AABB::contain(const Vector2r &p) const
{
    if ((min_.array() > p.array()).any()) return false;
    if ((max_.array() < p.array()).any()) return false;
    return true;
}

AABB &AABB::operator+=(const Vector2r &p)
{
    min_ = min_.cwiseMin(p);
    max_ = max_.cwiseMax(p);
    return *this;
}

AABB &AABB::operator+=(const AABB &other)
{
    min_ = min_.cwiseMin(other.min_);
    max_ = max_.cwiseMax(other.max_);
    return *this;
}

AABB AABB::operator+(const AABB &other) const
{
    AABB res(*this);
    return res += other;
}

real AABB::width() const { return max_[0] - min_[0]; }
real AABB::height() const { return max_[1] - min_[1]; }
real AABB::area() const { return width() * height(); }
real AABB::perimeter() const { return 2.0 * (width() + height()); }
real AABB::size() const { return (max_ - min_).squaredNorm(); }
real AABB::radius() const { return (max_ - min_).norm() * 0.5; }
Vector2r AABB::center() const { return (min_ + max_) * 0.5; }
real AABB::distance(const AABB &other, Vector2r *P, Vector2r *Q) const
{
    real result = 0;
    for (std::size_t i = 0; i < 2; ++i)
    {
        const real &amin = min_[i];
        const real &amax = max_[i];
        const real &bmin = other.min_[i];
        const real &bmax = other.max_[i];

        if (amin > bmax)
        {
            real delta = bmax - amin;
            result += delta * delta;
            if (P && Q)
            {
                (*P)[i] = amin;
                (*Q)[i] = bmax;
            }
        }
        else if (bmin > amax)
        {
            real delta = amax - bmin;
            result += delta * delta;
            if (P && Q)
            {
                (*P)[i] = amax;
                (*Q)[i] = bmin;
            }
        }
        else
        {
            if (P && Q)
            {
                if (bmin >= amin)
                {
                    real t = 0.5 * (amax + bmin);
                    (*P)[i] = t;
                    (*Q)[i] = t;
                }
                else
                {
                    real t = 0.5 * (amin + bmax);
                    (*P)[i] = t;
                    (*Q)[i] = t;
                }
            }
        }
    }

    return std::sqrt(result);
}

real AABB::distance(const AABB &other) const
{
    real result = 0;
    for (std::size_t i = 0; i < 2; ++i)
    {
        const real &amin = min_[i];
        const real &amax = max_[i];
        const real &bmin = other.min_[i];
        const real &bmax = other.max_[i];

        if (amin > bmax)
        {
            real delta = bmax - amin;
            result += delta * delta;
        }
        else if (bmin > amax)
        {
            real delta = amax - bmin;
            result += delta * delta;
        }
    }

    return std::sqrt(result);
}

bool AABB::equal(const AABB &other) const
{
    return min_.isApprox(other.min_, std::numeric_limits<real>::epsilon() * 100) &&
           max_.isApprox(other.max_, std::numeric_limits<real>::epsilon() * 100);
}

AABB &AABB::expand(const Vector2r &delta)
{
    min_ -= delta;
    max_ += delta;
    return *this;
}

AABB &AABB::expand(const AABB &core, real ratio)
{
    min_ = min_ * ratio - core.min_;
    max_ = max_ * ratio - core.max_;
    return *this;
}

AABB AABB::translate(const AABB &aabb, const Vector2r &t)
{
    AABB res(aabb);
    res.min_ += t;
    res.max_ += t;
    return res;
}
}  // namespace bp2d
}  // namespace cdl
