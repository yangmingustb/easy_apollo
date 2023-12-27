/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "gjk2d.h"

namespace cdl
{
namespace np2d
{
real GJK2D::Distance(const ShapeProxy2D &P, const ShapeProxy2D &Q, DistResult2D *result)
{
    result->clear();
    std::size_t k = 0;
    real norm2Wmax = 0;
    real tennorm = 0;

    Vector2r v(1.0, 0.0); /**< search direction */
    Vector2r w;           /**< CSO vertex boundary */
    real squared_v;

    localMinimum(P, Q, v);

    std::size_t p_idx, q_idx;

    do
    {
        squared_v = dot2D(v, v);
        k++;
        p_idx = Support2D(P, -v);
        q_idx = Support2D(Q, v);
        w = P.vertices[p_idx] - Q.vertices[q_idx];
        /** first exit condition */
        if ((squared_v - dot2D(v, w)) <= eps_rel * squared_v)
        {
            break;
        }

        /** 2nd exit condition */
        if (squared_v < eps_tol)
        {
            break;
        }
        /** add a new CSO vetex to simplex */
        simplex_.vertices.row(simplex_.num) = w;
        simplex_.pwids(simplex_.num) = p_idx;
        simplex_.qwids(simplex_.num) = q_idx;
        simplex_.num++;

        /** sub distance */
        SubDistance(simplex_, v);

        /** 3rd exit condition */
        for (std::size_t i = 0; i < simplex_.num; ++i)
        {
            tennorm = dot2D(simplex_.vertices.row(i), simplex_.vertices.row(i));
            if (tennorm > norm2Wmax)
            {
                norm2Wmax = tennorm;
            }
        }

        if (v.norm() <= eps_rel * eps_rel * norm2Wmax)
        {
            break;
        }

    } while ((simplex_.num != 3) && (k != max_iter));

    result->distance = std::sqrt(dot2D(v, v));
    result->iterations = k;

    for (std::size_t i = 0; i < simplex_.num; ++i)
    {
        result->p += simplex_.lambdas(i) * P.vertices[simplex_.pwids(i)];
        result->q += simplex_.lambdas(i) * Q.vertices[simplex_.qwids(i)];
    }
    /** status=seperation */
    if (result->distance > eps_rel)
    {
        result->status = CollisionStatus::SEPERATION;
        result->normal = (result->p - result->q).normalized();
    }
    else
    {
        /** point contact， */
        if (simplex_.num == 1)
        {
            result->status = CollisionStatus::POINT_CONTACT;
            getPointContactDirections(P, Q, result);
        }
        /** line contact or point contact */
        else if (simplex_.num == 2)
        {
            Vector2r p1, p2, q1, q2;
            p1 = P.vertices[simplex_.pwids(0)];
            p2 = P.vertices[simplex_.pwids(1)];
            q1 = P.vertices[simplex_.qwids(0)];
            q2 = P.vertices[simplex_.qwids(1)];
            if (dot2D(p1 - p2, p1 - p2) < eps_tol || dot2D(q1 - q2, q1 - q2) < eps_tol)
            {
                result->status = CollisionStatus::POINT_CONTACT;
                getPointContactDirections(P, Q, result);
            }
            else
            {
                result->status = CollisionStatus::LINE_CONTACT;
                result->normal =
                        ccwNormal2D(simplex_.vertices.row(1) - simplex_.vertices.row(0))
                                .normalized();
            }
        }
        /** return 3 vertices, so the status is point-contact, or line-contact,
         * or penetration */
        else
        {
            /** point-contact */
            if (simplex_.lambdas(0) + simplex_.lambdas(1) < eps_tol)
            {
                result->status = CollisionStatus::POINT_CONTACT;
                simplex_.pwids[0] = simplex_.pwids[2];
                simplex_.qwids[0] = simplex_.qwids[2];
                getPointContactDirections(P, Q, result);
            }

            /** line contact*/
            else if (simplex_.lambdas(0) * simplex_.lambdas(1) < eps_tol)
            {
                result->status = CollisionStatus::LINE_CONTACT;
                result->normal =
                        ccwNormal2D(simplex_.vertices.row(2) -
                                    simplex_.vertices.row(0) * simplex_.lambdas[0] -
                                    simplex_.vertices.row(1) * simplex_.lambdas[1])
                                .normalized();
            }
            /** penetration */
            else
            {
                result->status = CollisionStatus::PENETRATION;
            }
        }
    }
    return result->distance;
}  // Distance api

real GJK2D::Distance2(const ShapeProxy2D &P, const ShapeProxy2D &Q, DistResult2D *result)
{
    result->clear();
    std::size_t k = 0;
    real norm2Wmax = 0;
    real tennorm = 0;

    Vector2r v; /**< search direction */
    Vector2r w; /**< CSO vertex boundary */

    v = P.vertices[0] - Q.vertices[0];
    simplex_.num = 1;
    simplex_.vertices.row(0) = v;
    simplex_.pwids(0) = 0;
    simplex_.qwids(0) = 0;
    simplex_.lambdas(0) = 1.0;

    std::size_t p_idx, q_idx;
    real squared_v;
    do
    {
        squared_v = dot2D(v, v);
        k++;
        p_idx = Support2D(P, -v);
        q_idx = Support2D(Q, v);
        w = P.vertices[p_idx] - Q.vertices[q_idx];
        /** first exit condition */
        if ((squared_v - dot2D(v, w)) <= eps_tol)
        {
            break;
        }

        /** 2nd exit condition */
        if (squared_v < eps_tol)
        {
            break;
        }
        /** add a new CSO vetex to simplex */
        simplex_.vertices.row(simplex_.num) = w;
        simplex_.pwids(simplex_.num) = p_idx;
        simplex_.qwids(simplex_.num) = q_idx;
        simplex_.num++;
        /** sub distance */
        SubDistance(simplex_, v);
        /** 3rd exit condition */
        for (std::size_t i = 0; i < simplex_.num; ++i)
        {
            tennorm = dot2D(simplex_.vertices.row(i), simplex_.vertices.row(i));
            if (tennorm > norm2Wmax)
            {
                norm2Wmax = tennorm;
            }
        }

        if (dot2D(v, v) <= eps_rel * eps_rel * norm2Wmax)
        {
            break;
        }

    } while ((simplex_.num != 3) && (k != max_iter));

    result->distance = std::sqrt(dot2D(v, v));
    result->iterations = k;
    for (std::size_t i = 0; i < simplex_.num; ++i)
    {
        result->p += simplex_.lambdas(i) * P.vertices[simplex_.pwids(i)];
        result->q += simplex_.lambdas(i) * Q.vertices[simplex_.qwids(i)];
    }
    return result->distance;
}  // distance2 API

real GJK2D::Distance3(const ShapeProxy2D &P, const ShapeProxy2D &Q, DistResult2D *result)
{
    std::size_t k = 0;
    real norm2Wmax = 0;
    real tennorm = 0;

    Vector2r v(1.0, 0.0); /**< search direction */
    Vector2r w;           /**< CSO vertex boundary */

    std::size_t p_idx = Support2D(P, v);
    std::size_t q_idx = Support2D(Q, -v);
    v = P.vertices[p_idx] - Q.vertices[q_idx];
    simplex_.num = 1;
    simplex_.vertices.row(0) = v;
    simplex_.pwids(0) = p_idx;
    simplex_.qwids(0) = q_idx;
    simplex_.lambdas(0) = 1.0;

    do
    {
        real squared_v = dot2D(v, v);
        k++;
        p_idx = Support2D(P, -v);
        q_idx = Support2D(Q, v);
        w = P.vertices[p_idx] - Q.vertices[q_idx];
        /** first exit condition */
        if ((squared_v - dot2D(v, w)) <= eps_rel * squared_v)
        {
            break;
        }

        /** 2nd exit condition */
        if (squared_v < eps_tol)
        {
            break;
        }
        /** add a new CSO vetex to simplex */
        simplex_.vertices.row(simplex_.num) = w;
        simplex_.pwids(simplex_.num) = p_idx;
        simplex_.qwids(simplex_.num) = q_idx;
        simplex_.num++;
        /** sub distance */
        SubDistance(simplex_, v);
        /** 3rd exit condition */
        for (std::size_t i = 0; i < simplex_.num; ++i)
        {
            tennorm = dot2D(simplex_.vertices.row(i), simplex_.vertices.row(i));
            if (tennorm > norm2Wmax)
            {
                norm2Wmax = tennorm;
            }
        }

        if (dot2D(v, v) <= eps_rel * eps_rel * norm2Wmax)
        {
            break;
        }

    } while ((simplex_.num != 3) && (k != max_iter));

    result->distance = std::sqrt(dot2D(v, v));
    result->iterations = k;
    for (std::size_t i = 0; i < simplex_.num; ++i)
    {
        result->p += simplex_.lambdas(i) * P.vertices[simplex_.pwids(i)];
        result->q += simplex_.lambdas(i) * Q.vertices[simplex_.qwids(i)];
    }
    return result->distance;
}  // distance3 API

bool GJK2D::Collision(const ShapeProxy2D &P, const ShapeProxy2D &Q)
{
    std::size_t k = 0;
    real norm2Wmax = 0;
    real tennorm = 0;

    Vector2r v(1.0, 0.0); /**< search direction */
    Vector2r w;           /**< CSO vertex boundary */

    v = P.vertices[0] - Q.vertices[0];
    simplex_.num = 1;
    simplex_.vertices.row(0) = v;
    simplex_.pwids(0) = 0;
    simplex_.qwids(0) = 0;
    simplex_.lambdas(0) = 1.0;

    do
    {
        real squared_v = dot2D(v, v);
        k++;
        std::size_t p_idx = Support2D(P, -v);
        std::size_t q_idx = Support2D(Q, v);
        w = P.vertices[p_idx] - Q.vertices[q_idx];
        /** first exit condition, no collision */
        if (dot2D(v, w) > 0.0)
        {
            return false;
        }
        else
        {
            /** second exist condition, collision */
            if (simplex_.num == 2)
            {
                real a = cross2D(simplex_.vertices.row(0), w);
                real b = cross2D(simplex_.vertices.row(1), w);
                if (a * b <= 0.0)
                {
                    return true;
                }
            }
        }

        /** 3rd exit condition */
        if (squared_v < eps_tol)
        {
            return true;
        }
        /** add a new CSO vetex to simplex */
        simplex_.vertices.row(simplex_.num) = w;
        simplex_.pwids(simplex_.num) = p_idx;
        simplex_.qwids(simplex_.num) = q_idx;
        simplex_.num++;
        /** sub distance */
        SubDistance(simplex_, v);
        /** 3rd exit condition */
        for (std::size_t i = 0; i < simplex_.num; ++i)
        {
            tennorm = dot2D(simplex_.vertices.row(i), simplex_.vertices.row(i));
            if (tennorm > norm2Wmax)
            {
                norm2Wmax = tennorm;
            }
        }

        if (dot2D(v, v) <= eps_rel * eps_rel * norm2Wmax)
        {
            break;
        }

    } while ((simplex_.num != 3) && (k != max_iter));

    return true;
}

void GJK2D::setShapeEdgeNormal(Vector2r *vertices, uint32_t &size, Vector2r normals[])
{
    for (std::size_t i = 0; i < size; ++i)
    {
        std::size_t j = (i + 1 < size) ? i + 1 : 0;
        Vector2r edge = vertices[j] - vertices[i];
        normals[i] = cwNormal2D(edge).normalized();
    }
}

bool GJK2D::traversalRayCast(RayCastRequest *input, RayCastResult *output)
{
    const int num = input->shape.size;
    Vector2r normals[num];
    setShapeEdgeNormal(input->shape.vertices, input->shape.size, normals);

    real lower = 0.0;
    real upper = input->distance_limit;
    int lower_index = -1;
    int upper_index = 0;
    real numerator, denominator;
    for (int i = 0; i < num; ++i)
    {
        numerator = dot2D(normals[i], input->shape.vertices[i] - input->source);
        denominator = dot2D(normals[i], input->direction);

        if (fabs(denominator) < eps_tol)
        {
            if (numerator < 0.0)
            {
                output->status = RayCastStatus::NO_HIT;
                return false;
            }
        }
        else
        {
            if (denominator < 0.0 && numerator < lower * denominator)
            {
                lower = numerator / denominator;
                lower_index = i;
            }
            else if (denominator > 0.0 && numerator < upper * denominator)
            {
                upper = numerator / denominator;
                upper_index = i;
            }
        }
        if (upper < lower || input->distance_limit < lower)
        {
            output->status = RayCastStatus::NO_HIT;
            return false;
        }
    }
    if (lower <= 0.0)
    {
        output->status = RayCastStatus::SOURCE_IS_INSIDE;
        output->lambda_hit = upper;
        output->hit_normal = normals[upper_index];
        output->hit_distance = upper;
        output->hit_spot = input->source + upper * input->direction;
        output->hit_edge_id = upper_index;
        return true;
    }
    output->lambda_hit = lower;
    output->hit_normal = normals[lower_index];
    output->hit_distance = lower;
    output->hit_spot = input->source + lower * input->direction;

    if (lower_index > -1)
    {
        if (lower > input->max_lambda)
        {
            output->status = RayCastStatus::WILL_HIT;
            output->hit_edge_id = lower_index;
            return false;
        }
        output->hit_edge_id = lower_index;
        output->status = RayCastStatus::HIT;
        return true;
    }
    output->status = RayCastStatus::NO_HIT;
    return false;
};

bool GJK2D::GJKBasedRayCast(RayCastRequest *input, RayCastResult *output)
{
    Vector2r x = input->source;
    Vector2r delta_x;
    Vector2r w; /**< the vector of p-x */
    Vector2r p;
    real lambda = 0.0;
    real delta_lambda;
    Vector2r normal;
    simplex_.num = 0;

    Vector2r v;

    v = input->shape.vertices[0] - x;

    std::size_t i, j;
    real vv, vw, vr;
    std::size_t p_idx;

    for (i = 0; i < max_iter; ++i)
    {
        vv = dot2D(v, v);
        if (vv < eps_tol)
        {
            break;
        }

        p_idx = Support2D(input->shape, -v);
        p = input->shape.vertices[p_idx];
        w = p - x;

        vw = dot2D(v, w);
        if (vw > 0.0)
        {
            vr = dot2D(v, input->direction);
            if (vr <= 0.0)
            {
                output->status = RayCastStatus::NO_HIT;
                return false;
            }
            else
            {
                delta_lambda = vw / vr;
                lambda += delta_lambda;
                delta_x = delta_lambda * input->direction;
                x += delta_x;
                normal = -v;
                /** simplex translation, no need to flush */
                for (j = 0; j < simplex_.num; ++j)
                {
                    simplex_.vertices.row(j) -= delta_x;
                }
            }
        }

        if (simplex_.num > 0)
        {
            if (vv - vw < eps_tol)
            {
                SubDistance(simplex_, v);
                if (v.norm() < eps_rel)
                {
                    break;
                }
                else
                {
                    output->status = RayCastStatus::NO_HIT;
                    output->iterations = i;
                    return false;
                }
            }
        }
        simplex_.vertices.row(simplex_.num) = p - x;
        simplex_.pwids(simplex_.num) = 0;
        simplex_.qwids(simplex_.num) = p_idx;
        simplex_.num++;
        SubDistance(simplex_, v);

        if (simplex_.num == 3)
        {
            break;
        }

        if (lambda > input->distance_limit)
        {
            output->status = RayCastStatus::NO_HIT;
            return false;
        }
    }

    /** If the current lower bound distance is larger than the maximum
     *  raycasting distance */
    if (lambda > input->max_lambda)
    {
        if (dot2D(v, w) < eps_rel)
        {
            output->status = RayCastStatus::WILL_HIT;
            return false;
        }
        output->status = RayCastStatus::NO_HIT;
        return false;
    }
    /** If the origin was inside the shape, we return no hit */
    if (lambda < eps_rel)
    {
        output->status = RayCastStatus::SOURCE_IS_INSIDE;
        return false;
    }

    /** A raycast hit has been found, we fill in the raycast info */
    output->hit_distance = lambda;
    output->lambda_hit = lambda;
    output->hit_normal = normal.normalized();
    output->hit_spot = x;
    output->status = RayCastStatus::HIT;
    return true;
}

bool GJK2D::shapeCast(ShapeCastResult *output, ShapeCastRequest *input)
{
    const ShapeProxy2D &P = input->proxyA;
    const ShapeProxy2D &Q = input->proxyB;

    const Vector2r r = input->translationB - input->translationA;

    Vector2r n(0.0, 0.0);
    real time = 0.0;
    real delta_time = 0.0; /**< time increment */
    real last_time = 0.0;
    Vector2r s(0.0, 0.0);
    real norm2Wmax = 0.0;
    real tennorm = 0.0;

    /** Initial simplex */
    simplex_.num = 0;

    Vector2r v = P.vertices[0] - Q.vertices[0];

    real squared_v, vp, vr;
    std::size_t p_idx;
    std::size_t q_idx;
    std::size_t iter;
    std::size_t i;
    Vector2r p;
    Vector2r p_moving;
    squared_v = dot2D(v, v);
    for (iter = 0; iter < max_iter; ++iter)
    {
        if (squared_v < eps_tol)
        {
            break;
        }
        /** Support in direction -v (A - B) */
        p_idx = Support2D(P, -v);
        q_idx = Support2D(Q, v);
        p = P.vertices[p_idx] - Q.vertices[q_idx];
        p_moving = p - s;

        /** Intersect ray with plane */
        vp = dot2D(v, p);
        vr = dot2D(v, r);

        if (vp > time * vr)
        {
            /** v is the seperating axis */
            if (vr <= 0.0)
            {
                output->collided = false;
                output->iterations = iter;
                return false;
            }
            last_time = time;

            time = vp / vr;
            delta_time = time - last_time;
            if (time > 1.0)
            {
                output->collided = false;
                output->iterations = iter;
                return false;
            }
            s = time * r;
            n = -v;
            /** simplex translation, no need to flush */
            for (i = 0; i < simplex_.num; ++i)
            {
                simplex_.vertices.row(i) -= delta_time * r;
            }
        }
        if (simplex_.num > 0)
        {
            if (squared_v - dot2D(p_moving, v) < eps_tol)
            {
                SubDistance(simplex_, v);
                if (v.norm() < eps_rel)
                {
                    break;
                }
                else
                {
                    output->collided = false;
                    output->iterations = iter;
                    return false;
                }
            }
        }

        simplex_.vertices.row(simplex_.num) = p - s;
        simplex_.pwids(simplex_.num) = p_idx;
        simplex_.qwids(simplex_.num) = q_idx;
        simplex_.num++;

        SubDistance(simplex_, v);
        if (simplex_.num == 3)
        {
            break;
        }

        for (i = 0; i < simplex_.num; ++i)
        {
            tennorm = dot2D(simplex_.vertices.row(i), simplex_.vertices.row(i));
            if (tennorm > norm2Wmax)
            {
                norm2Wmax = tennorm;
            }
        }
        squared_v = dot2D(v, v);
        if (squared_v <= eps_tol * norm2Wmax)
        {
            break;
        }
    }

    output->collided = true;
    output->distanceA = time * input->translationA.norm();
    output->distanceB = time * input->translationB.norm();

    output->normalA = n.normalized();
    output->clear();
    for (i = 0; i < simplex_.num; ++i)
    {
        output->collision_pointA += simplex_.lambdas(i) * P.vertices[simplex_.pwids(i)];
        output->collision_pointB += simplex_.lambdas(i) * Q.vertices[simplex_.qwids(i)];
    }

    output->iterations = iter;
    return true;
}

}  // namespace np2d
}  // namespace cdl
