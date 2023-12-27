/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once
#include <algorithm>
#include <iostream>
#include <vector>
#include "collision_geometry2d.h"
#include "gjk2d.h"

namespace cdl
{
namespace bp2d
{
class CDL_EXPORT Convex : public CollisionGeometry
{
public:
    /**
     * The Convex geometry assumes that the input data vertices and faces do
     * not change through the life of the object.
     * \param  vertices : The positions of the polytope vertices.
     */
    Convex(const Vector2r *vertices, const uint32_t size);
    Convex(const Convex &other);
    Convex();

    void setConvex(const Vector2r *vertices, const uint32_t size);

    ~Convex() = default;

    void updateConvex(const Transform2r &tf);

    /** Computes AABB in the geometry's canonical frame. */
    void computeLocalAABB() override;

    /** Gets node type: a convex polytope. */
    NODE_TYPE getNodeType() const override;

    /** Gets the vertex positions in the geometry's frame G. */
    void getVertices(Vector2r *vertices, uint32_t *size)
    {
        vertices = vertices_;
        for (size_t i = 0; i < size_; i++)
        {
            vertices[i] = vertices_[i];
        }

        *size = size_;
    }

    /** A point guaranteed to be on the interior of the convex polytope */
    const Vector2r &getInteriorPoint() const { return interior_point_; }

    /** Gets the vertices of some convex shape which can bound this shape in a
     * specific configuration */
    void getBoundVertices(cdl::np2d::ShapeProxy2D *proxy) const;

public:
    Vector2r vertices_[cdl::MAX_CONVEX_VERTEX_NUM];

    uint32_t size_;

    Vector2r interior_point_;
};
}  // namespace bp2d
}  // namespace cdl
