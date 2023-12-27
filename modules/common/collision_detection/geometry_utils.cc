/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "geometry_utils.h"

namespace cdl
{
void updatePose(Transform3r& future, const Transform3r& current,
                real delta_length, real delta_yaw)
{
    real x1, y1, t1;
    auto q = cdl::rot3ToQuat(current.linear());
    real yaw = cdl::getYaw(q);
    if (std::fabs(delta_yaw) > 1e-4)
    {
        real R = std::fabs(delta_length / delta_yaw);
        t1 = yaw + std::copysign(M_PI / 2, delta_yaw);
        x1 = current.translation()(0) + R * std::cos(t1);
        y1 = current.translation()(1) + R * std::sin(t1);

        cdl::real next_yaw = yaw + delta_yaw;
        cdl::setYaw(future, next_yaw);

        cdl::real t2 = next_yaw - std::copysign(M_PI / 2, delta_yaw);
        future.translation()(0) = x1 + std::cos(t2) * R;
        future.translation()(1) = y1 + std::sin(t2) * R;
        future.translation()(2) = 0;
    }
    else
    {
        future.translation()(0) =
                current.translation()(0) + delta_length * std::cos(yaw);
        future.translation()(1) =
                current.translation()(1) + delta_length * std::sin(yaw);
        future.translation()(2) = 0;
        future.linear() = current.linear();
    }
}

Transform3r tf2dToTf3d(const Transform2r& tf)
{
    Transform3r tf3D;
    tf3D.linear() << tf.linear().row(0)(0), tf.linear().row(0)(1), 0.0,
            tf.linear().row(1)(0), tf.linear().row(1)(1), 0.0, 0.0, 0.0, 1.0;
    tf3D.translation() << tf.translation()(0), tf.translation()(1), 0.0;
    return tf3D;
}

Transform2r tf3dToTf2d(const Transform3r& tf)
{
    Transform2r tf2D;
    tf2D.linear() << tf.linear().row(0)(0), tf.linear().row(0)(1),
            tf.linear().row(1)(0), tf.linear().row(1)(1);
    tf2D.translation() << tf.translation()(0), tf.translation()(1);
    return tf2D;
}

}  // namespace cdl