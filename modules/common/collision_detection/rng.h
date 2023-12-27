/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#include <cassert>
#include <iostream>
#include <random>

#include "constants.h"
#include "seed.h"

namespace cdl
{
/**
 * \brief Random number generation. An instance of this class cannot be used by
 * multiple threads at once (member functions are not const). However, the
 * constructor is thread safe and different instances can be used safely in any
 * number of threads. It is also guaranteed that all created instances will have
 * a different random seed.
 */
class CDL_EXPORT RNG
{
public:
    /** Always sets a different random seed */
    RNG();

    /** Generate a random real between 0 and 1 */
    real uniform01();

    /** \brief Generate a random real within given bounds:
     * [lower_bound,upper_bound)
     */
    real uniformReal(real lower_bound, real upper_bound);

    /** Generate a random integer within given bounds:
     * [lower_bound, upper_bound]
     */
    int uniformInt(int lower_bound, int upper_bound);

    /** Generate a random boolean */
    bool uniformBool();

    /** Generate a random real using a normal distribution with mean 0 and
     * variance 1*/
    real gaussian01();

    /** Generate a random real using a normal distribution with given mean and
     * variance */
    real gaussian(real mean, real stddev);

    /**
     * \brief Generate a random real using a half-normal distribution. The value
     * is within specified bounds [\e r_min, r_max], but with a bias towards
     * r_max. The function is implemended using a Gaussian distribution with
     * mean at r_max - r_min. The distribution is 'folded' around r_max axis
     * towards r_min. The variance of the distribution is (r_max - r_min) focus.
     * The higher the focus, the more probable it is that generated numbers are
     * close to r_max.
     */
    real halfNormalReal(real r_min, real r_max, real focus = 3.0);

    /** \brief Generate a random integer using a half-normal distribution. The
     * value is within specified bounds ([\e r_min, r_max]), but with a bias
     * towards r_max. The function is implemented on top of halfNormalReal()
     */
    int halfNormalInt(int r_min, int r_max, real focus = 3.0);

    /** \brief Uniform random unit quaternion sampling. The computed value has
     * the order (x,y,z,w)
     */
    void quaternion(real value[4]);

    /** \brief Uniform random sampling of Euler roll-pitch-yaw angles, each in
     * the range [-pi, pi). The computed value has the order (roll, pitch, yaw)
     */
    void eulerRPY(real value[3]);

    /** \brief Uniform random sample on a disk with radius from r_min to r_max
     */
    void disk(real r_min, real r_max, real& x, real& y);

    /** \brief Uniform random sample in a ball with radius from r_min to r_max
     */
    void ball(real r_min, real r_max, real& x, real& y, real& z);

    /** \brief realet the seed for random number generation. Use this function
     * to
     *  ensure the same sequence of random numbers is generated.
     */
    static void setSeed(std::uint_fast32_t seed);

    /** \brief Get the seed used for random number generation. Passing the
     *  returned value to setrealeed() at a subsequent execution of the code
     *  will ensure deterministic (repeatable) behaviour. Useful for debugging.
     */
    static std::uint_fast32_t getSeed();

private:
    std::mt19937 generator_;
    std::uniform_real_distribution<> uniDist_;
    std::normal_distribution<> normalDist_;

};  // RNG class

}  // namespace cdl
