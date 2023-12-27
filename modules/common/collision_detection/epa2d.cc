/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "epa2d.h"


namespace cdl
{
namespace np2d
{
void EPA2D::calculateClosestEdge(size_t* closest_edge_idx) const
{
    *closest_edge_idx = 0;
    real closest_distance = epa_edges_[0].distance_to_origin;
    for (std::size_t i = 1; i < vertice_num; ++i)
    {
        if (epa_edges_[i].distance_to_origin < closest_distance)
        {
            closest_distance = epa_edges_[i].distance_to_origin;
            *closest_edge_idx = i;
        }
    }
}

real EPA2D::originToEdgeDistance(const Vector2r& point1,
                                 const Vector2r& point2) const
{
    /* orthogonal is the clockwise normal of point 1 and point 2 */
    Vector2r orthogonal = cwNormal2D(point2 - point1);
    orthogonal.normalize();
    return dot2D(point1, orthogonal);
}

void EPA2D::enableCounterClockwise(Simplex2D& simplex_)
{
    Vector2r ab = simplex_.vertices.row(1) - simplex_.vertices.row(0);
    Vector2r ac = simplex_.vertices.row(2) - simplex_.vertices.row(0);
    real aCrossB = cross2D(ab, ac);
    if (aCrossB > 0)  /** counter-clockwise */
    {
        for (std::size_t i = 0; i < simplex_.num; ++i)
        {
            std::size_t j = (i + 1 == simplex_.num) ? 0 : i + 1;

            epa_edges_[i].p_id(0) = simplex_.pwids(i);
            epa_edges_[i].p_id(1) = simplex_.pwids(j);
            epa_edges_[i].q_id(0) = simplex_.qwids(i);
            epa_edges_[i].q_id(1) = simplex_.qwids(j);
            epa_edges_[i].distance_to_origin = originToEdgeDistance(
                simplex_.vertices.row(i), simplex_.vertices.row(j));
        }
    }

    /** if clockwise, enable the simplex counter-clockwise */
    else
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            std::size_t j = (i + 2 > 2) ? i - 1 : i + 2;

            epa_edges_[i].p_id(0) = simplex_.pwids(i);
            epa_edges_[i].p_id(1) = simplex_.pwids(j);
            epa_edges_[i].q_id(0) = simplex_.qwids(i);
            epa_edges_[i].q_id(1) = simplex_.qwids(j);
            epa_edges_[i].distance_to_origin = originToEdgeDistance(
                simplex_.vertices.row(i), simplex_.vertices.row(j));
        }
    }
    vertice_num = simplex_.num;
}

real EPA2D::pointToEdgeDistance(const Vector2r& point, const Vector2r& edge_p0,
                                const Vector2r& edge_p1) const
{
    if (edge_p0 == edge_p1)
    {
        return (point - edge_p0).norm();
    }
    Vector2r orthogonal = ccwNormal2D(edge_p1 - edge_p0);
    orthogonal.normalize();
    return fabs(dot2D(edge_p0 - point, orthogonal));
}

void EPA2D::zeroOrderSimplexExpanding(const ShapeProxy2D& P,
                                      const ShapeProxy2D& Q,
                                      Simplex2D& gjk_simplex) const
{
    const Vector2r& point_a = gjk_simplex.vertices.row(0);
    Vector2r diff;
    for (size_t i = 0; i < P.size; ++i)
    {
        for (size_t j = 0; j < Q.size; ++j)
        {
            Vector2r v = P.vertices[i] - Q.vertices[j];

            Vector2i new_point_id;
            new_point_id(0) = Support2D(P, v);
            new_point_id(1) = Support2D(Q, -v);
            Vector2r point =
                P.vertices[new_point_id(0)] - Q.vertices[new_point_id(1)];
            diff = point - point_a;
            if (dot2D(diff, diff) < 1e-10)
            {
                continue;
            }
            else
            {
                gjk_simplex.vertices.row(1) = point;
                gjk_simplex.pwids(1) = i;
                gjk_simplex.qwids(1) = j;
                gjk_simplex.num++;
                return;
            }
        }
    }
    return;
}

void EPA2D::firstOrderSimplexExpanding(const ShapeProxy2D& P,
                                       const ShapeProxy2D& Q,
                                       Simplex2D& gjk_simplex,
                                       DistResult2D* result)
{
    const Vector2r& point_a = gjk_simplex.vertices.row(0);
    const Vector2r& point_b = gjk_simplex.vertices.row(1);
    Vector2r orthogonal =
        cwNormal2D(point_b - point_a);  /** generate clockwise simplex */

    Vector2i new_point_id;
    new_point_id(0) = Support2D(P, orthogonal);
    new_point_id(1) = Support2D(Q, -orthogonal);
    Vector2r point = P.vertices[new_point_id(0)] - Q.vertices[new_point_id(1)];

    if (pointToEdgeDistance(point, point_a, point_b) < cdl::constants::eps())
    {
        result->normal = orthogonal.normalized();
        result->distance = 0.0;

        Vector2r lambda;
        Vector2r e12;

        e12 = point_b - point_a;

        real d12_2 = -dot2D(point_a, e12);
        real d12_1 = dot2D(point_b, e12);

        real inv_d12 = 1.0 / (d12_1 + d12_2);
        lambda(0) = d12_1 * inv_d12;
        lambda(1) = 1 - lambda(0);
        result->p = lambda(0) * (P.vertices[gjk_simplex.pwids(0)]) +
                    lambda(1) * (P.vertices[gjk_simplex.pwids(1)]);
        result->q = lambda(0) * (Q.vertices[gjk_simplex.qwids(0)]) +
                    lambda(1) * (Q.vertices[gjk_simplex.qwids(1)]);
        return;
    }

    Vector2r orthogonal_right_hand = ccwNormal2D(point_b - point_a);

    Vector2i new_point_id_right;
    new_point_id_right(0) = Support2D(P, orthogonal_right_hand);
    new_point_id_right(1) = Support2D(Q, -orthogonal_right_hand);
    Vector2r point_right =
        P.vertices[new_point_id_right(0)] - Q.vertices[new_point_id_right(1)];

    if (pointToEdgeDistance(point_right, point_a, point_b) <
        cdl::constants::eps())
    {
        result->normal = orthogonal_right_hand.normalized();
        result->distance = 0.0;

        Vector2r lambda;
        Vector2r e12;

        e12 = point_b - point_a;

        real d12_2 = -dot2D(point_a, e12);
        real d12_1 = dot2D(point_b, e12);

        real inv_d12 = 1.0 / (d12_1 + d12_2);
        lambda(0) = d12_1 * inv_d12;
        lambda(1) = 1 - lambda(0);
        result->p = lambda(0) * (P.vertices[gjk_simplex.pwids(0)]) +
                    lambda(1) * (P.vertices[gjk_simplex.pwids(1)]);
        result->q = lambda(0) * (Q.vertices[gjk_simplex.qwids(0)]) +
                    lambda(1) * (Q.vertices[gjk_simplex.qwids(1)]);
        return;
    }
    gjk_simplex.vertices.row(2) = point_right;
    gjk_simplex.pwids(2) = new_point_id_right(0);
    gjk_simplex.qwids(2) = new_point_id_right(1);
    gjk_simplex.num++;

    for (std::size_t i = 0; i < gjk_simplex.num; ++i)
    {
        std::size_t j = (i + 1 == gjk_simplex.num) ? 0 : i + 1;

        epa_edges_[i].p_id(0) = gjk_simplex.pwids(i);
        epa_edges_[i].p_id(1) = gjk_simplex.pwids(j);
        epa_edges_[i].q_id(0) = gjk_simplex.qwids(i);
        epa_edges_[i].q_id(1) = gjk_simplex.qwids(j);
        epa_edges_[i].distance_to_origin = originToEdgeDistance(
            gjk_simplex.vertices.row(i), gjk_simplex.vertices.row(j));
    }
    vertice_num = gjk_simplex.num;
}

void EPA2D::gjkSimplexToEpaSimplex(const ShapeProxy2D& P, const ShapeProxy2D& Q,
                                   Simplex2D& gjk_simplex, DistResult2D* result)
{
    if (gjk_simplex.num == 1)
    {
        zeroOrderSimplexExpanding(P, Q, gjk_simplex);
        if (gjk_simplex.num == 1)
        {
            return;
        }

        firstOrderSimplexExpanding(P, Q, gjk_simplex, result);
        if (gjk_simplex.num == 2)
        {
            return;
        }
    }
    else if (gjk_simplex.num == 2)
    {
        firstOrderSimplexExpanding(P, Q, gjk_simplex, result);
        if (gjk_simplex.num == 2)
        {
            return;
        }
    }
    else
    {
        return;
    }
};

void EPA2D::Penetration(const ShapeProxy2D& P, const ShapeProxy2D& Q,
                        Simplex2D& gjk_simplex, DistResult2D& epa_result)
{
    epa_result.clear();
    enableCounterClockwise(gjk_simplex);

    real closest_distance;
    std::size_t closest_edge_idx;
    Vector2r new_vertex;

    SimplexEdge2D* edge;
    Vector2r point_1, point_2;
    Vector2r dir;
    Vector2i new_vertex_pq_id;
    real projection;
    std::size_t i;

    for (i = 0; i < max_iter; ++i)
    {
        calculateClosestEdge(&closest_edge_idx);

        edge = epa_edges_ + closest_edge_idx;
        closest_distance = edge->distance_to_origin;

        point_1 = P.vertices[edge->p_id(0)] - Q.vertices[edge->q_id(0)];
        point_2 = P.vertices[edge->p_id(1)] - Q.vertices[edge->q_id(1)];
        dir = ccwNormal2D(point_1 - point_2);
        new_vertex_pq_id(0) = Support2D(P, dir);
        new_vertex_pq_id(1) = Support2D(Q, -dir);
        new_vertex =
            P.vertices[new_vertex_pq_id(0)] - Q.vertices[new_vertex_pq_id(1)];

        dir.normalize();
        projection = dot2D(new_vertex, dir);
        if (projection - closest_distance < eps_tol)
        {
            epa_result.normal = dir;
            epa_result.distance = -closest_distance;

            Vector2r lambda;
            Vector2r e12 = point_2 - point_1;

            real d12_2 = -dot2D(point_1, e12);
            real d12_1 = dot2D(point_2, e12);

            real inv_d12 = 1.0 / (d12_1 + d12_2);
            lambda(0) = d12_1 * inv_d12;
            lambda(1) = 1 - lambda(0);
            epa_result.p = lambda(0) * (P.vertices[edge->p_id(0)]) +
                           lambda(1) * (P.vertices[edge->p_id(1)]);
            epa_result.q = lambda(0) * (Q.vertices[edge->q_id(0)]) +
                           lambda(1) * (Q.vertices[edge->q_id(1)]);
            epa_result.iterations = i;
            return;
        }

        /** expand simplex */

        epa_edges_[vertice_num].p_id(0) = new_vertex_pq_id(0);
        epa_edges_[vertice_num].q_id(0) = new_vertex_pq_id(1);
        epa_edges_[vertice_num].p_id(1) = edge->p_id(1);
        epa_edges_[vertice_num].q_id(1) = edge->q_id(1);
        epa_edges_[vertice_num].distance_to_origin =
            originToEdgeDistance(new_vertex, point_2);
        vertice_num++;

        epa_edges_[closest_edge_idx].p_id(1) = new_vertex_pq_id(0);
        epa_edges_[closest_edge_idx].q_id(1) = new_vertex_pq_id(1);
        epa_edges_[closest_edge_idx].distance_to_origin =
            originToEdgeDistance(point_1, new_vertex);

    }
};

void EPA2D::Penetration2(const ShapeProxy2D& P, const ShapeProxy2D& Q,
                         Simplex2D* gjk_simplex, DistResult2D& epa_result)
{

    if (gjk_simplex->num < 3)
    {
        gjkSimplexToEpaSimplex(P, Q, *gjk_simplex, &epa_result);
        if (gjk_simplex->num < 3)
        {
            return;
        }
    }
    else
    {
        enableCounterClockwise(*gjk_simplex);
    }

    real closest_distance;
    std::size_t closest_edge_idx;
    Vector2r new_vertex;

    SimplexEdge2D* edge;
    Vector2r point_1, point_2;
    Vector2r dir;
    Vector2i new_vertex_pq_id;
    real projection;
    std::size_t i;
    for (i = 0; i < max_iter; ++i)
    {
        calculateClosestEdge(&closest_edge_idx);

        edge = epa_edges_ + closest_edge_idx;
        closest_distance = edge->distance_to_origin;

        point_1 = P.vertices[edge->p_id(0)] - Q.vertices[edge->q_id(0)];
        point_2 = P.vertices[edge->p_id(1)] - Q.vertices[edge->q_id(1)];
        dir = ccwNormal2D(point_1 - point_2);
        new_vertex_pq_id(0) = Support2D(P, dir);
        new_vertex_pq_id(1) = Support2D(Q, -dir);
        new_vertex =
            P.vertices[new_vertex_pq_id(0)] - Q.vertices[new_vertex_pq_id(1)];

        dir.normalize();
        projection = dot2D(new_vertex, dir);
        if (projection - closest_distance < eps_tol)
        {
            epa_result.normal = dir;
            epa_result.distance = -closest_distance;

            Vector2r lambda;
            Vector2r e12 = point_2 - point_1;

            real d12_2 = -dot2D(point_1, e12);
            real d12_1 = dot2D(point_2, e12);

            real inv_d12 = 1.0 / (d12_1 + d12_2);
            lambda(0) = d12_1 * inv_d12;
            lambda(1) = 1 - lambda(0);
            epa_result.p = lambda(0) * (P.vertices[edge->p_id(0)]) +
                           lambda(1) * (P.vertices[edge->p_id(1)]);
            epa_result.q = lambda(0) * (Q.vertices[edge->q_id(0)]) +
                           lambda(1) * (Q.vertices[edge->q_id(1)]);
            epa_result.iterations = i;
            return;
        }

        epa_edges_[vertice_num].p_id(0) = new_vertex_pq_id(0);
        epa_edges_[vertice_num].q_id(0) = new_vertex_pq_id(1);
        epa_edges_[vertice_num].p_id(1) = edge->p_id(1);
        epa_edges_[vertice_num].q_id(1) = edge->q_id(1);
        epa_edges_[vertice_num].distance_to_origin =
            originToEdgeDistance(new_vertex, point_2);
        vertice_num++;

        epa_edges_[closest_edge_idx].p_id(1) = new_vertex_pq_id(0);
        epa_edges_[closest_edge_idx].q_id(1) = new_vertex_pq_id(1);
        epa_edges_[closest_edge_idx].distance_to_origin =
            originToEdgeDistance(point_1, new_vertex);

    }
};

}  // namespace np2D
}  // namespace cdl
