/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "box2d.h"

namespace cdl
{
namespace bp2d
{
Box::Box(const Vector2r *vertices) : CollisionGeometry()
{
    for (size_t i = 0; i < 4; ++i)
    {
        vertices_[i] = vertices[i];
    }
}

Box::Box(const Vector2r &side) : CollisionGeometry()
{
    real a = side(0) * 0.5;
    real b = side(1) * 0.5;
    vertices_[0] = Vector2r(a, -b);
    vertices_[1] = Vector2r(a, b);
    vertices_[2] = Vector2r(-a, b);
    vertices_[3] = Vector2r(-a, -b);
}

Box::Box(const real &x, const real &y) : CollisionGeometry()
{
    real a = x * 0.5;
    real b = y * 0.5;
    vertices_[0] = Vector2r(a, -b);
    vertices_[1] = Vector2r(a, b);
    vertices_[2] = Vector2r(-a, b);
    vertices_[3] = Vector2r(-a, -b);
}

Box::Box() : CollisionGeometry() {}

void Box::computeLocalAABB()
{
    this->aabb_local.min_.setConstant(std::numeric_limits<real>::max());
    this->aabb_local.max_.setConstant(-std::numeric_limits<real>::max());

    for (size_t i = 0; i < 4; ++i)
    {
        this->aabb_local += vertices_[i];
    }

    this->aabb_center = this->aabb_local.center();
    this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

void Box::updateBox(const Transform2r &tf)
{
    for (size_t i = 0; i < 4; ++i)
    {
        vertices_[i] = tf * vertices_[i];
    }
}

void Box::setBox(const real &x, const real &y, const Transform2r &tf)
{
    real a = x * 0.5;
    real b = y * 0.5;
    vertices_[0] = tf * Vector2r(a, -b);
    vertices_[1] = tf * Vector2r(a, b);
    vertices_[2] = tf * Vector2r(-a, b);
    vertices_[3] = tf * Vector2r(-a, -b);
}

void Box::setBox(const Vector2r *vertices)
{
    for (size_t i = 0; i < 4; ++i)
    {
        vertices_[i] = vertices[i];
    }
}

NODE_TYPE Box::getNodeType() const { return GEOM_BOX; }

void Box::getBoundVertices(cdl::np2d::ShapeProxy2D *proxy) const
{
    proxy->size = 4;
    for (size_t i = 0; i < 4; ++i)
    {
        proxy->vertices[i] = vertices_[i];
    }
}

Vector2r Box::getSide() const
{
    Vector2r side;
    side(0) = (vertices_[1] - vertices_[2]).norm();
    side(1) = (vertices_[1] - vertices_[0]).norm();
    return side;
}

real Box::side(const int32 &i)
{
    if (i == 0)
    {
        return (vertices_[1] - vertices_[2]).norm();
    }
    else
    {
        return (vertices_[1] - vertices_[0]).norm();
    }
}

}  // namespace bp2d
}  // namespace cdl
