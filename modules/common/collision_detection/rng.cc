/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "rng.h"

namespace cdl
{
RNG::RNG() : generator_(Seed::getNextSeed()), uniDist_(0, 1), normalDist_(0, 1)
{
}
real RNG::uniform01() { return uniDist_(generator_); }
real RNG::uniformReal(real lower_bound, real upper_bound)
{
    assert(lower_bound <= upper_bound);

    return (upper_bound - lower_bound) * uniDist_(generator_) + lower_bound;
}

int32 RNG::uniformInt(int32 lower_bound, int32 upper_bound)
{
    int32 r = (int32)floor(
            uniformReal((real)lower_bound, (real)(upper_bound) + 1.0));

    return (r > upper_bound) ? upper_bound : r;
}

bool RNG::uniformBool() { return uniDist_(generator_) <= 0.5; }
real RNG::gaussian01() { return normalDist_(generator_); }
real RNG::gaussian(real mean, real stddev)
{
    return normalDist_(generator_) * stddev + mean;
}

real RNG::halfNormalReal(real r_min, real r_max, real focus)
{
    assert(r_min <= r_max);

    const auto mean = r_max - r_min;
    auto v = gaussian(mean, mean / focus);

    if (v > mean) v = 2.0 * mean - v;

    auto r = v >= 0.0 ? v + r_min : r_min;

    return r > r_max ? r_max : r;
}

int RNG::halfNormalInt(int r_min, int r_max, real focus)
{
    int r = (int)std::floor(
            halfNormalReal((real)r_min, (real)(r_max) + 1.0, focus));

    return (r > r_max) ? r_max : r;
}

void RNG::quaternion(real value[])
{
    auto x0 = uniDist_(generator_);
    auto r1 = std::sqrt(1.0 - x0), r2 = std::sqrt(x0);
    auto t1 = 2.0 * constants::pi() * uniDist_(generator_);
    auto t2 = 2.0 * constants::pi() * uniDist_(generator_);
    auto c1 = std::cos(t1);
    auto s1 = std::sin(t1);
    auto c2 = std::cos(t2);
    auto s2 = std::sin(t2);
    value[0] = s1 * r1;
    value[1] = c1 * r1;
    value[2] = s2 * r2;
    value[3] = c2 * r2;
}

void RNG::eulerRPY(real value[])
{
    value[0] = constants::pi() * (2.0 * uniDist_(generator_) - 1.0);
    value[1] =
            std::acos(1.0 - 2.0 * uniDist_(generator_)) - constants::pi() / 2.0;
    value[2] = constants::pi() * (2.0 * uniDist_(generator_) - 1.0);
}

void RNG::disk(real r_min, real r_max, real& x, real& y)
{
    auto a = uniform01();
    auto b = uniform01();
    auto r = std::sqrt(a * r_max * r_max + (1 - a) * r_min * r_min);
    auto theta = 2 * constants::pi() * b;
    x = r * std::cos(theta);
    y = r * std::sin(theta);
}

void RNG::ball(real r_min, real r_max, real& x, real& y, real& z)
{
    auto a = uniform01();
    auto b = uniform01();
    auto c = uniform01();
    auto r = std::pow(a * std::pow(r_max, 3) + (1 - a) * std::pow(r_min, 3),
                      1 / 3.0);
    auto theta = std::acos(1 - 2 * b);
    auto phi = 2 * constants::pi() * c;

    auto costheta = std::cos(theta);
    auto sintheta = std::sin(theta);
    auto cosphi = std::cos(phi);
    auto sinphi = std::sin(phi);
    x = r * costheta;
    y = r * sintheta * cosphi;
    z = r * sintheta * sinphi;
}

void RNG::setSeed(uint_fast32_t seed)
{
    if (Seed::isFirstSeedGenerated())
    {
        std::cerr << "Random number generation already started. Changing seed "
                     "now "
                  << "will not lead to deterministic sampling." << std::endl;
    }

    if (seed == 0)
    {
        std::cerr << "Random generator seed cannot be 0. Using 1 instead."
                  << std::endl;
        Seed::setUserSetSeed(1);
    }
    else
    {
        Seed::setUserSetSeed(seed);
    }
}

uint_fast32_t RNG::getSeed() { return Seed::getFirstSeed(); }
}  // namespace cdl
