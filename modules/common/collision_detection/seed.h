/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#include <cstdint>
#include "macro.h"

namespace cdl
{
class CDL_EXPORT Seed
{
public:
    static bool isFirstSeedGenerated();
    static std::uint_fast32_t getUserSetSeed();
    static void setUserSetSeed(std::uint_fast32_t seed);
    static std::uint_fast32_t getFirstSeed();
    static std::uint_fast32_t getNextSeed();

protected:
    Seed();
    static Seed& getInstance();
    /** The seed the user asked for (cannot be 0)*/
    std::uint_fast32_t userSetSeed;
    /** Flag indicating whether the first seed has already been generated or
     * not*/
    bool firstSeedGenerated;
    /** The value of the first seed */
    std::uint_fast32_t firstSeedValue;
};  // class Seed

}  // namespace cdl
