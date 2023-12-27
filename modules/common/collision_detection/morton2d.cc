/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "morton2d.h"

namespace cdl
{
namespace bp2d
{
int32 quantize(real x, int32 n)
{
    return std::max(
            std::min(static_cast<int32>(x * n), static_cast<int32>(n - 1)),
            static_cast<int32>(0));
}

int32 morton_code(int32 x, int32 y)
{
    x = (x ^ (x << 8)) & 0x00ff00ff; /**< 1 */
    x = (x ^ (x << 4)) & 0x0f0f0f0f; /**< (2) */
    x = (x ^ (x << 2)) & 0x33333333; /**< (3) */
    x = (x ^ (x << 1)) & 0x55555555; /**< (4) */

    y = (y ^ (y << 8)) & 0x00ff00ff; /**< (1) */
    y = (y ^ (y << 4)) & 0x0f0f0f0f; /**< (2) */
    y = (y ^ (y << 2)) & 0x33333333; /**< (3) */
    y = (y ^ (y << 1)) & 0x55555555; /**< (4) */

    return x | (y << 1);
}

uint64 morton_code60(int32 x, int32 y)
{
    int32 lo_x = x & 1023u;
    int32 lo_y = y & 1023u;
    int32 hi_x = x >> 10u;
    int32 hi_y = y >> 10u;

    return (uint64(morton_code(hi_x, hi_y)) << 30) |
           uint64(morton_code(lo_x, lo_y));
}

morton_functoru32r::morton_functoru32r(const AABB &bbox)
    : base(bbox.min_),
      inv(1.0 / (bbox.max_[0] - bbox.min_[0]),
          1.0 / (bbox.max_[1] - bbox.min_[1]))
{
}

int32 morton_functoru32r::operator()(const Vector2r &point) const
{
    int32 x = quantize((point[0] - base[0]) * inv[0], 1024u);
    int32 y = quantize((point[1] - base[1]) * inv[1], 1024u);

    return morton_code(x, y);
}

morton_functoru64r::morton_functoru64r(const AABB &bbox)
    : base(bbox.min_),
      inv(1.0 / (bbox.max_[0] - bbox.min_[0]),
          1.0 / (bbox.max_[1] - bbox.min_[1]))
{
}

uint64 morton_functoru64r::operator()(const Vector2r &point) const
{
    int32 x = quantize((point[0] - base[0]) * inv[0], 1u << 20);
    int32 y = quantize((point[1] - base[1]) * inv[1], 1u << 20);

    return morton_code60(x, y);
}

template <size_t N>
morton_functor<std::bitset<N>>::morton_functor(const AABB &bbox)
    : base(bbox.min_),
      inv(1.0 / (bbox.max_[0] - bbox.min_[0]),
          1.0 / (bbox.max_[1] - bbox.min_[1]))
{
}

template <size_t N>
std::bitset<N> morton_functor<std::bitset<N>>::operator()(
        const Vector2r &point) const
{
    real x = (point[0] - base[0]) * inv[0];
    real y = (point[1] - base[1]) * inv[1];
    int start_bit = bits() - 1;
    std::bitset<N> bset;

    x *= 2;
    y *= 2;

    for (size_t i = 0; i < bits() / 3; ++i)
    {
        bset[start_bit--] = ((y < 1) ? 0 : 1);
        bset[start_bit--] = ((x < 1) ? 0 : 1);
        x = ((x >= 1) ? 2 * (x - 1) : 2 * x);
        y = ((y >= 1) ? 2 * (y - 1) : 2 * y);
    }

    return bset;
}

}  // namespace bp2d
}  // namespace cdl
