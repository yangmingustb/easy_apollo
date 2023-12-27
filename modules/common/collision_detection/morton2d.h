/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#include <bitset>
#include "aabb2d.h"
#include "types.h"

namespace cdl
{
namespace bp2d
{
CDL_EXPORT
int32 quantize(real x, int32 n);

/**
 * \brief compute 30 bit morton code
 */
CDL_EXPORT
int32 morton_code(int32 x, int32 y);

/**
 * \brief compute 60 bit morton code
 */
CDL_EXPORT
uint64 morton_code60(int32 x, int32 y);

struct CDL_EXPORT morton_functoru32r
{
    morton_functoru32r(const AABB &bbox);

    int32 operator()(const Vector2r &point) const;

    const Vector2r base;
    const Vector2r inv;

    static constexpr size_t bits() { return 30; }
};

struct CDL_EXPORT morton_functoru64r
{
    morton_functoru64r(const AABB &bbox);

    uint64 operator()(const Vector2r &point) const;

    const Vector2r base;
    const Vector2r inv;

    static constexpr size_t bits() { return 60; }
};

template <typename T>
struct CDL_EXPORT morton_functor
{
};

template <size_t N>
struct CDL_EXPORT morton_functor<std::bitset<N>>
{
    static_assert(N % 3 == 0, "Number of bits must be a multiple of 3");

    morton_functor(const AABB &bbox);

    std::bitset<N> operator()(const Vector2r &point) const;

    const Vector2r base;
    const Vector2r inv;

    static constexpr size_t bits() { return N; }
};

}  // namespace bp2d
}  // namespace cdl
