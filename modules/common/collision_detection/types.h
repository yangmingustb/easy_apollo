/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#include "macro.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <float.h>

#define UOS_CDL_USE_DOUBLE_PRECISION (1)

#define ESP_REL_FLOAT64 (1.0e-5)
#define ESP_REL_FLOAT32 (1.0e-3)

namespace cdl
{
/** use a general type to switch between float and double */
#ifdef UOS_CDL_USE_DOUBLE_PRECISION
typedef double real;
#define real_max DBL_MAX;
#define ESP_REL ESP_REL_FLOAT64
#else
typedef float real;
#define real_max FLT_MAX;
#define ESP_REL ESP_REL_FLOAT32
#endif

using int64 = std::int64_t;
using uint64 = std::uint64_t;
using int32 = std::int32_t;
using uint32 = std::uint32_t;

template <typename S> using Vector2 = Eigen::Matrix<S, 2, 1>;

template <typename S> using Vector3 = Eigen::Matrix<S, 3, 1>;

template <typename S> using Vector4 = Eigen::Matrix<S, 4, 1>;

template <typename S> using Vector6 = Eigen::Matrix<S, 6, 1>;

template <typename S> using Vector7 = Eigen::Matrix<S, 7, 1>;

template <typename S, int N> using VectorN = Eigen::Matrix<S, N, 1>;

template <typename S> using VectorX = Eigen::Matrix<S, Eigen::Dynamic, 1>;

template <typename S> using Matrix3 = Eigen::Matrix<S, 3, 3>;

template <typename S> using Matrix43 = Eigen::Matrix<S, 4, 3>; /* for simplex */

template <typename S> using MatrixX3 = Eigen::Matrix<S, Eigen::Dynamic, 3>;

template <typename S> using Matrix2 = Eigen::Matrix<S, 2, 2>;

template <typename S> using Quaternion = Eigen::Quaternion<S>;

template <typename S>
using Transform3 = Eigen::Transform<S, 3, Eigen::Isometry>;

template <typename S>
using Transform2 = Eigen::Transform<S, 2, Eigen::Isometry>;

template <typename S> using Translation3 = Eigen::Translation<S, 3>;

template <typename S> using Translation2 = Eigen::Translation<S, 2>;

template <typename S> using AngleAxis = Eigen::AngleAxis<S>;

template <typename S> using Matrix32 = Eigen::Matrix<S, 3, 2>; // simplex2d

using PointType = Vector2<real>;
using SimplexType = Matrix32<real>;

/** real types */
using Vector4r = Vector4<real>;
using Vector3r = Vector3<real>;
using Vector2r = Vector2<real>;
template <int32_t N> using VectorNr = VectorN<real, N>;
using VectorXr = VectorX<real>;
using Matrix3r = Matrix3<real>;
using Vector3i = Eigen::Matrix<int32_t, 3, 1>;
using Vector4i = Eigen::Matrix<int32_t, 4, 1>;
using Vector2i = Eigen::Matrix<int32_t, 2, 1>;
using Matrix43r = Matrix43<real>;
using MatrixX3r = MatrixX3<real>;
using Matrix2r = Matrix2<real>;
using Quaternionr = Quaternion<real>;
using Transform3r = Transform3<real>;
using Transform2r = Transform2<real>;
using Translation3r = Translation3<real>;
using Translation2r = Translation2<real>;
using AngleAxisr = AngleAxis<real>;

template <typename _Tp>
using aligned_vector = std::vector<_Tp, Eigen::aligned_allocator<_Tp>>;

} // namespace cdl
