/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once
#include "types.h"

namespace cdl
{
CDL_ALWAYS_INLINE
void setYaw(cdl::Transform3r& tf, real yaw)
{
    real cs = std::cos(yaw);
    real ss = std::sin(yaw);
    tf.linear() << cs, -ss, 0.0, ss, cs, 0.0, 0.0, 0.0, 1.0;
}

CDL_ALWAYS_INLINE cdl::Quaternionr rot3ToQuat(const cdl::Matrix3r& r)
{
    cdl::Quaternionr q;
    cdl::real rbcm = (r(2, 1) - r(1, 2)) * (r(2, 1) - r(1, 2));
    cdl::real racm = (r(2, 0) - r(0, 2)) * (r(2, 0) - r(0, 2));
    cdl::real rabm = (r(1, 0) - r(0, 1)) * (r(1, 0) - r(0, 1));
    cdl::real rbcp = (r(2, 1) + r(1, 2)) * (r(2, 1) + r(1, 2));
    cdl::real racp = (r(2, 0) + r(0, 2)) * (r(2, 0) + r(0, 2));
    cdl::real rabp = (r(1, 0) + r(0, 1)) * (r(1, 0) + r(0, 1));
    cdl::real wsqr = (r(0, 0) + r(1, 1) + r(2, 2) + 1) *
                     (r(0, 0) + r(1, 1) + r(2, 2) + 1);
    cdl::real xsqr = (r(0, 0) - r(1, 1) - r(2, 2) + 1) *
                     (r(0, 0) - r(1, 1) - r(2, 2) + 1);
    cdl::real ysqr = (r(1, 1) - r(0, 0) - r(2, 2) + 1) *
                     (r(1, 1) - r(0, 0) - r(2, 2) + 1);
    cdl::real zsqr = (r(2, 2) - r(0, 0) - r(1, 1) + 1) *
                     (r(2, 2) - r(0, 0) - r(1, 1) + 1);
    q.w() = 0.25 * std::sqrt(wsqr + rbcm + rabm + racm);
    q.x() = 0.25 * std::sqrt(xsqr + rbcm + rabp + racp);
    q.y() = 0.25 * std::sqrt(ysqr + racm + rabp + rbcp);
    q.z() = 0.25 * std::sqrt(zsqr + rabm + racp + rbcp);
    return q;
}

CDL_ALWAYS_INLINE real getYaw(const Quaternionr& q)
{
    /** yaw (z-axis rotation) */
    real siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    real cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());

    return std::atan2(siny_cosp, cosy_cosp);
}
/** update pose according to nonholonomic constraints */
CDL_EXPORT
void updatePose(cdl::Transform3r& future, const cdl::Transform3r& current,
                real delta_length, real delta_yaw);

Transform3r tf2dToTf3d(const Transform2r& tf);

Transform2r tf3dToTf2d(const Transform3r& tf);

}  // namespace cdl
