/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "seed.h"
#include <chrono>
#include <mutex>
#include <random>

namespace cdl
{
bool Seed::isFirstSeedGenerated() { return getInstance().firstSeedGenerated; }
uint_fast32_t Seed::getUserSetSeed() { return getInstance().userSetSeed; }
void Seed::setUserSetSeed(uint_fast32_t seed)
{
    getInstance().userSetSeed = seed;
}

uint_fast32_t Seed::getFirstSeed()
{
    /* Compute the first seed to be used; this function should be called only
     * once */
    static std::mutex fsLock;
    std::unique_lock<std::mutex> slock(fsLock);

    if (getInstance().firstSeedGenerated) return getInstance().firstSeedValue;

    if (getInstance().userSetSeed != 0)
    {
        getInstance().firstSeedValue = getInstance().userSetSeed;
    }
    else
    {
        getInstance().firstSeedValue = static_cast<std::uint_fast32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now() -
                        std::chrono::system_clock::time_point())
                        .count());
    }

    getInstance().firstSeedGenerated = true;

    return getInstance().firstSeedValue;
}

uint_fast32_t Seed::getNextSeed()
{
    static std::mutex rngMutex;
    std::unique_lock<std::mutex> slock(rngMutex);
    static std::ranlux24_base sGen;
    static std::uniform_int_distribution<> sDist(1, 1000000000);

    return sDist(sGen);
}

Seed::Seed() : userSetSeed(0), firstSeedGenerated(false), firstSeedValue(0) {}

Seed& Seed::getInstance()
{
    static Seed seed;

    return seed;
}
}  // namespace cdl
