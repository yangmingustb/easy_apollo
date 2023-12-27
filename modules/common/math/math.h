
#pragma once

#include <algorithm>
#include <cmath>

namespace apollo
{
#define apollo_abs(x) std::abs(x)

#define apollo_pow(x, y) std::pow(x, y)
#define apollo_pow2(x) ((x) * (x))
#define apollo_sqrt(x) std::sqrt(x)

#define apollo_max(x, y) ((x) > (y) ? (x) : (y))
#define apollo_min(x, y) ((x) < (y) ? (x) : (y))

#define apollo_sin(x) std::sin(x)
#define apollo_cos(x) std::cos(x)
#define apollo_tan(x) std::tan(x)
#define apollo_asin(x) std::asin(x)
#define apollo_acos(x) std::acos(x)
#define apollo_atan(x) std::atan(x)
#define apollo_atan2(y, x) std::atan2(y, x)
#define apollo_sincos(x, s, c) std::sincos(x, s, c)

#define apollo_floor(x) std::floor(x)
#define apollo_ceil(x) std::ceil(x)
#define apollo_round(x) std::round(x)
#define apollo_fabs(x) std::fabs(x)
#define apollo_fmod(x, y) std::fmod(x, y)

#define apollo_rand() std::rand()
#define uos_rand_shift(min, max) \
    min + apollo_rand() / (RAND_MAX / (max - min + 1) + 1)

#define apollo_srand(seed) std::srand(seed)
#define apollo_math_log(x) std::log(x)
#define apollo_exp(x) std::exp(x)

#define apollo_i32abs(x) std::abs(x)

#define apollo_PI (3.14159265358979323846)
#define apollo_HALF_PI (1.57079632679489661923)
#define apollo_TWO_PI (6.28318530717958647692)
#define apollo_SQRT_PI (1.77245385090551602729)
#define apollo_SQRT_PI_INV (0.56418958354775628695)

#define apollo_FLOAT64_EPSILON (1e-9)

#define apollo_deg2rad(x) ((x)*apollo_PI / 180.0)
#define apollo_rad2deg(x) ((x)*180.0 / apollo_PI)

#define apollo_cvt_kmh_to_ms(vel) (((vel)*1000) / 3600)
#define apollo_cvt_ms_to_kmh(vel) (((vel)*3600) / 1000)

#define apollo_fequal(x, y)    (apollo_fabs((x) - (y)) < apollo_FLOAT64_EPSILON)
#define apollo_fgreater(x, y) (((x) > (y)) && !apollo_fequal(x, y))
#define apollo_fless(x, y) (((x) < (y)) && !apollo_fequal(x, y))

#define apollo_fequal_32(x, y) (apollo_fabs((x) - (y)) < apollo_FLOAT64_EPSILON)
#define apollo_fgreater_32(x, y) (((x) > (y)) && !apollo_fequal_32(x, y))
#define apollo_fless_32(x, y) (((x) < (y)) && !apollo_fequal_32(x, y))

#define apollo_fix(x) (apollo_fgreater(x, 0.0) ? apollo_floor(x) : apollo_ceil(x))
#define apollo_sign(x) (apollo_fgreater(x, 0.0) ? 1 : (apollo_fless(x, 0.0) ? -1 : 0))

#define INVALID_IDX (-1)

}  // namespace apollo
