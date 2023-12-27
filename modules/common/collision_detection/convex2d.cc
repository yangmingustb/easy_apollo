/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project droot for full license information.
 */

#include "convex2d.h"
#include "cdl_math.h"

namespace cdl
{
namespace bp2d
{
bool vectorCmp(const Vector2r &a, const Vector2r &b)
{
    if (a(0) < b(0)) return true;
    if ((a(0) - b(0)) == 0.0 && a(1) < b(1))
    {
        return true;
    }
    return false;
}

Convex::Convex(const Vector2r *vertices, const uint32_t size)
    : CollisionGeometry()
{
    Vector2r sum = Vector2r::Zero();
    for (size_t i = 0; i < size; ++i)
    {
        vertices_[i] = vertices[i];
        sum += vertices_[i];
    }
    size_ = size;

    if (size_ != 0)
    {
        interior_point_ = sum * static_cast<real>(1.0 / size_);
    }
    else
    {
        interior_point_ = Vector2r(0.0, 0.0);
        std::cout << __FILE__ << "@" << __func__ << " line: " << __LINE__
                  << "invalid vertices_ " << size_ << std::endl;
    }
}

void Convex::setConvex(const Vector2r *vertices, const uint32_t size)
{
    Vector2r sum = Vector2r::Zero();
    for (size_t i = 0; i < size; ++i)
    {
        vertices_[i] = vertices[i];
        sum += vertices_[i];
    }
    size_ = size;

    if (size_ != 0)
    {
        interior_point_ = sum * static_cast<real>(1.0 / size_);
    }
    else
    {
        interior_point_ = Vector2r(0.0, 0.0);
        std::cout << __FILE__ << "@" << __func__ << " line: " << __LINE__
                  << "invalid vertices_ " << size_ << std::endl;
    }
}

Convex::Convex() : CollisionGeometry() {}

void Convex::updateConvex(const Transform2r &tf)
{
    Vector2r sum = Vector2r::Zero();
    for (size_t i = 0; i < size_; ++i)
    {
        vertices_[i] = tf * vertices_[i];
        sum += vertices_[i];
    }

    if (size_ != 0)
    {
        interior_point_ = sum * static_cast<real>(1.0 / size_);
    }
    else
    {
        interior_point_ = Vector2r(0.0, 0.0);
        std::cout << __FILE__ << "@" << __func__ << " line: " << __LINE__
                  << "invalid vertices_ " << size_ << std::endl;
    }
}

void Convex::computeLocalAABB()
{
    this->aabb_local.min_.setConstant(std::numeric_limits<real>::max());
    this->aabb_local.max_.setConstant(-std::numeric_limits<real>::max());

    for (size_t i = 0; i < size_; ++i)
    {
        this->aabb_local += vertices_[i];
    }

    this->aabb_center = this->aabb_local.center();
    this->aabb_radius = (this->aabb_local.min_ - this->aabb_center).norm();
}

NODE_TYPE Convex::getNodeType() const { return GEOM_CONVEX; }

void Convex::getBoundVertices(cdl::np2d::ShapeProxy2D *proxy) const
{
    proxy->size = size_;

    for (std::size_t i = 0; i < size_; ++i)
    {
        proxy->vertices[i] = vertices_[i];
    }
}
}  // namespace bp2d
}  // namespace cdl
