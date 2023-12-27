/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "collision_data2d.h"
#include "geometry_utils.h"
#include "rng.h"
#include "types.h"

enum class Status
{
    Separated,
    Near,
    Penetrated
};

class DataGenerator
{
public:
    using real = cdl::real;
    using tf3r = Eigen::Transform<real, 3, Eigen::Isometry>;
    using Vector3r = cdl::Vector3r;
    using Quaternionr = cdl::Quaternionr;

    cdl::RNG rng_;

    std::vector<cdl::Vector3r> rect_vertices;
    std::vector<cdl::Vector3r> panta_vertices;
    tf3r rect_tf;
    tf3r panta_tf;
    real rect_yaw;
    real panta_yaw;
    Vector3r rect_center;
    Vector3r panta_center;
    real rect_r;
    real panta_r;
    Status status;

public:
    DataGenerator(Status st) : status(st)
    {
        SetPolygons();
        SetTransforms(st);
    }
    ~DataGenerator(){};

    void SetPolygons()
    {
        rect_vertices.push_back(Vector3r(1, -1, 0));
        rect_vertices.push_back(Vector3r(1, 3, 0));
        rect_vertices.push_back(Vector3r(-1, 3, 0));
        rect_vertices.push_back(Vector3r(-1, -1, 0));

        panta_vertices.push_back(Vector3r(1.0, -2.0, 0));
        panta_vertices.push_back(Vector3r(2.0, 1.0, 0));
        panta_vertices.push_back(Vector3r(0, 3.0, 0));
        panta_vertices.push_back(Vector3r(-2.0, 1.0, 0));
        panta_vertices.push_back(Vector3r(-1.0, -2.0, 0));

        rect_center = GetAverage(rect_vertices);
        rect_r = GetRadius(rect_center, rect_vertices);

        panta_center = GetAverage(panta_vertices);
        panta_r = GetRadius(panta_center, panta_vertices);
    }

    void SetTransforms(Status st)
    {
        rect_tf.setIdentity();
        panta_tf.setIdentity();

        auto p = RandomPosition();
        rect_tf.translation() = p;
        rect_yaw = RandomYaw();
        cdl::setYaw(rect_tf, rect_yaw);

        auto offset = RandomOffset(st, rect_r, panta_r, 10.0);
        panta_tf.translation() = p + offset;
        panta_yaw = RandomYaw();
        cdl::setYaw(panta_tf, panta_yaw);
    }

    Vector3r GetAverage(const std::vector<Vector3r>& vertices)
    {
        Vector3r center = Vector3r::Zero();
        for (const auto& v : vertices)
        {
            center += v;
        }
        center = center / vertices.size();
        return center;
    }

    real GetRadius(const Vector3r& center,
                   const std::vector<Vector3r>& vertices)
    {
        real r = 0;
        for (const auto& v : vertices)
        {
            auto len = center - v;
            real radius = len(0) * len(0) + len(1) * len(1);
            if (radius > r)
            {
                r = radius;
            }
        }
        return std::sqrt(r);
    }

    Vector3r RandomPosition()
    {
        real x = rng_.uniformReal(0, 10.0);
        real y = rng_.uniformReal(0, 10.0);
        return Vector3r(x, y, 0);
    }

    real RandomYaw()
    {
        real yaw = rng_.uniformReal(-M_PI, M_PI);
        yaw = std::atan2(std::sin(yaw), std::cos(yaw));
        return yaw;
    }

    Vector3r RandomOffset(Status st, real r1, real r2, real max_dist)
    {
        real delta_lower;
        real delta_upper;
        switch (st)
        {
            case Status::Separated:
                delta_lower = r1 + r2;
                delta_upper = delta_lower + max_dist;
                break;
            case Status::Near:
                delta_lower = r1 >= r2 ? r1 - r2 : r2 - r1;
                delta_upper = r1 + r2;
                break;
            case Status::Penetrated:
                delta_lower = 0;
                delta_upper = r1 >= r2 ? r1 - r2 : r2 - r1;
                break;
            default:
                delta_lower = 0;
                delta_upper = 1;
                break;
        }

        real r = rng_.uniformReal(delta_lower, delta_upper);
        real theta = RandomYaw();
        return Vector3r(r * cos(theta), r * sin(theta), 0);
    }
};
