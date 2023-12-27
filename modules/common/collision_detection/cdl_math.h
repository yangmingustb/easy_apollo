/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once
#include <iostream>
#include "types.h"

namespace cdl
{
/** Returns the clockwise normal of a vector.(x,y) ---> (y,-x) */
inline Vector2r cwNormal2D(const Vector2r &v) { return Vector2r(v[1], -v[0]); }

/** Returns the counter-clockwise normal of a vector. (x,y) ---> (-y,x) */
inline Vector2r ccwNormal2D(const Vector2r &v) { return Vector2r(-v[1], v[0]); }

inline real cross2D(const Vector2r &p1, const Vector2r &p2)
{
    return p1[0] * p2[1] - p1[1] * p2[0];
}

inline real dot2D(const Vector2r &a, const Vector2r &b)
{
    return (a[0] * b[0] + a[1] * b[1]);
}

inline Vector2r min(const Vector2r &a, const Vector2r &b)
{
    return Vector2r(std::min(a[0], b[0]), std::min(a[1], b[1]));
}

inline void cdl_min(Vector2r *min_, const Vector2r &a, const Vector2r &b)
{
    (*min_)[0] = std::min(a[0], b[0]);
    (*min_)[1] = std::min(a[1], b[1]);
}

inline real min(const real a, const real b)
{
    if (a < b)
    {
        return a;
    }
    return b;
}

inline Vector2r max(const Vector2r &a, const Vector2r &b)
{
    return Vector2r(std::max(a[0], b[0]), std::max(a[1], b[1]));
}

inline void cdl_max(Vector2r *max_, const Vector2r &a, const Vector2r &b)
{
    (*max_)[0] = std::max(a[0], b[0]);
    (*max_)[1] = std::max(a[1], b[1]);
}

inline real max(const real a, const real b)
{
    if (a < b)
    {
        return b;
    }
    return a;
}

}  // namespace cdl
