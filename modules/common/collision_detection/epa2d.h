/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#include <iostream>
#include "cdl_math.h"
#include "constants.h"
#include "gjk2d.h"
#include "types.h"

namespace cdl
{
namespace np2d
{
struct SimplexEdge2D
{
    Vector2i p_id;
    Vector2i q_id;
    real distance_to_origin;
};

class EPA2D
{
public:
    static constexpr std::size_t max_iter = 25;
    /** tolarence */
    static constexpr real eps_tol = 1e-5;

    static constexpr std::size_t max_vertices = 20;
    SimplexEdge2D epa_edges_[max_vertices];
    std::size_t vertice_num;

public:
    void calculateClosestEdge(size_t* closest_edge_idx) const;

    real originToEdgeDistance(const Vector2r& point1,
                              const Vector2r& point2) const;

    void enableCounterClockwise(Simplex2D& simplex_);

    real pointToEdgeDistance(const Vector2r& point, const Vector2r& edge_p0,
                             const Vector2r& edge_p1) const;

    /**
     * \brief
     *
     * \param[in] P
     * \param[in] Q
     * \param[in/out] gjk_simplex: gjk_simplex only contains one vertice, and
     * this vertice must be origin(0,0). So the COLLISION2D_STATUS is
     * POINT_CONTACT OR LINE CONTACT or PENETRATION. We will use the vertice
     * index to determing the status.
     */
    void zeroOrderSimplexExpanding(const ShapeProxy2D& P, const ShapeProxy2D& Q,
                                   Simplex2D& gjk_simplex) const;

    /**
     * \brief when gjk_result.simplex contains two vertice, origin point must
     * lies in simplex's edge, not the vertice. But if gjk_result.simplex
     * contains one vertice, and the simplex has been expanded by
     * zeroOrderSimplexExpanding() function, then origin maybe lies in a vertice
     * of the gjk simplex.
     *
     * \param P
     * \param Q
     * \param gjk_simplex
     */
    void firstOrderSimplexExpanding(const ShapeProxy2D& P,
                                    const ShapeProxy2D& Q,
                                    Simplex2D& gjk_simplex,
                                    DistResult2D* result);

    void gjkSimplexToEpaSimplex(const ShapeProxy2D& P, const ShapeProxy2D& Q,
                                Simplex2D& gjk_simplex, DistResult2D* result);

    /**
     * \brief EPA algorithm penetration depth API. This API receives complete
     * gjk-simplex(if two polygons are overlapped, then the number of the
     * vertices in the gjk-simplex is 3)
     *
     * \param[in] P
     * \param[in] Q
     * \param[in] gjk_simplex
     * \param[out] epa_result
     */
    void Penetration(const ShapeProxy2D& P, const ShapeProxy2D& Q,
                     Simplex2D& gjk_simplex, DistResult2D& epa_result);

    /**
     * \brief EPA API2. The penetration2 api can resolve degenerate inputs,
     * even if the number of the simplex is 1, or 2.
     *
     * \param[in] P
     * \param[in] Q
     * \param[in] gjk_simplex
     * \param[out] epa_result
     */
    void Penetration2(const ShapeProxy2D& P, const ShapeProxy2D& Q,
                      Simplex2D* gjk_simplex, DistResult2D& epa_result);
};  //   epa2d class
}  // namespace np2d
}  // namespace cdl
