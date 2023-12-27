/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#include "cdl_math.h"
#include "constants.h"
#include "types.h"
#include "modules/common/math/math.h"

#define UOS_MAX_POLYGON_VERTEX_NUM (12)

namespace cdl
{
namespace np2d
{
enum struct CollisionStatus
{
    SEPERATION, /**< Shapes seperation */
    CONTACT,
    POINT_CONTACT, /**< Point contact between two shapes */
    LINE_CONTACT,  /**< Line contact between two shapes */
    PENETRATION,   /**< Shapes are penetrating */
    GJK_FAILED,    /**< GJK phase fail */
    EPA_FAILED,    /**< EPA phase fail */
    STATUS_NUM
};

struct Simplex2D
{
    std::size_t num;        /**< number of simplex */
    SimplexType vertices;   /**< simplex data */
    Vector3r lambdas;       /**< Barycentric coordinates for each vertex */
    Vector3i pwids;         /**< labels of simplex vertex */
    Vector3i qwids;         /**< labels of simplex vertex */
    std::size_t iterations; /**< iterations of gjk algorithms */
    Simplex2D()
        : num(0),
          vertices(SimplexType::Zero()),
          lambdas(Vector3r::Zero()),
          pwids(Vector3i::Zero()),
          qwids(Vector3i::Zero()),
          iterations(0){};
};

struct ShapeProxy2D
{
    PointType vertices[UOS_MAX_POLYGON_VERTEX_NUM];
    uint32_t size;
};

struct DistResult2D
{
    PointType p;
    PointType q;
    /** if line contact, return the normal of the touching edge, the direction
     * is arbitrary.
     * if point contact, return two directions. They are the upper and lower
     * bounds of the separable direction domain. And the direction of rotation
     * is counterclockwise. normal is the lower bound, second_direction is the
     * upper bound.
     * if seperated, return the difference of the two witness points, q-->p,
     * if penetrated, return the difference of the witness points, q-->p.
     */
    Vector2r normal; /**< need to be normalized */
    Vector2r second_direction;
    real distance;
    std::size_t iterations;
    CollisionStatus status;
    DistResult2D()
        : p(Vector2r::Zero()),
          q(Vector2r::Zero()),
          normal(Vector2r::Zero()),
          distance(0.0),
          iterations(0),
          status(CollisionStatus::SEPERATION){};
    void clear()
    {
        p = Vector2r::Zero();
        q = Vector2r::Zero();
    }
};

struct RayCastRequest
{
    Vector2r source;
    Vector2r direction; /**< normalized */
    real max_lambda;    /**< the distance you expect to explore */
    ShapeProxy2D shape;
    real distance_limit = 1000.0; /**< raycast distance limit */
};

enum class RayCastStatus
{
    HIT,              /**< The ray hits with a polygon. */
    WILL_HIT,         /**< The ray will hit with a polygon if
                       * you increase the length of the ray. */
    NO_HIT,           /**< Absolutly no hit */
    SOURCE_IS_INSIDE, /**< The source point is inside a polygon */
    RAYCAST_STATUS_NUM
};

struct RayCastResult
{
    Vector2r hit_spot;
    Vector2r hit_normal; /**< normalized */
    real hit_distance;
    real lambda_hit;
    std::size_t iterations;
    RayCastStatus status;
    int hit_edge_id;
    RayCastResult()
        : hit_spot(Vector2r::Zero()),
          hit_normal(Vector2r::Zero()),
          hit_distance(0.0),
          lambda_hit(0.0),
          iterations(0),
          status(RayCastStatus::NO_HIT),
          hit_edge_id(-1){};
};

struct ShapeCastRequest
{
    ShapeProxy2D proxyA;
    ShapeProxy2D proxyB;
    Vector2r translationA; /**< no need normalized, translationA = end-start */
    Vector2r translationB; /**< no need normalized */
};

/** if two polygons are collided along their translation direction, the status
 * of the shape cast result is collided, then return the witness, normal and
 * distance. The returned normal at the hit spot is the normal of the last
 * supporting plane that clipped the ray. If the objects A and B are
 * intersecting at time t = 0 , then of course the ray is never clipped, and the
 * algorithm terminates returning a zero hit spot and normal. This all makes
 * perfect sense since if the objects are already intersecting then the normal
 * at the first time of impact is not defined. if the status is no collision,
 * the value of witness, normal and distance is no meaning.
 */
struct ShapeCastResult
{
    bool collided;
    Vector2r collision_pointA;
    Vector2r collision_pointB;
    Vector2r normalA;
    Vector2r normalB;
    real distanceA; /**< the moving distance of A and B is equal if they are
                       moving */
    real distanceB; /**< the moving distance is 0 if one polygon is static */
    uint8_t iterations;
    ShapeCastResult()
        : collision_pointA(Vector2r::Zero()),
          collision_pointB(Vector2r::Zero()){};
    void clear()
    {
        collision_pointA = Vector2r::Zero();
        collision_pointB = Vector2r::Zero();
    }
};

#if 0
/** This is a traverse support version, no bug */
inline std::size_t Support2D(const ShapeProxy2D &body, const Vector2r &v)
{
    int32_t better = -1;
    real maxs;
    maxs = dot2D(body.vertices[0], v);
    for (std::size_t i = 0; i < body.size; ++i)
    {
        real s = dot2D(body.vertices[i], v);
        if (s > maxs)
        {
            maxs = s;
            better = i;
        }
    }

    if (better != -1)
    {
        return better;
    }
    else
    {
        return 0;
    }
}
#endif

inline size_t getClimbDirection(const ShapeProxy2D &body, const Vector2r &v,
                                real *value)
{
    size_t next_id = 2;
    size_t last_id = body.size - 2;
    real next, last, best;
    size_t best_index;

    while (next_id <= last_id)
    {
        next = dot2D(v, body.vertices[next_id]);
        if (next > *value)
        {
            best = next;
            best_index = next_id;
            for (size_t i = next_id + 1; i < body.size; ++i)
            {
                *value = dot2D(v, body.vertices[i]);
                if (*value < best)
                {
                    return best_index;
                }
                else if (*value > best)
                {
                    best = *value;
                    best_index = i;
                }
                else
                {
                    continue;
                }
            }
            return best_index;
        }

        last = dot2D(v, body.vertices[last_id]);
        if (last > *value)
        {
            best = last;
            best_index = last_id;
            for (size_t i = last_id - 1; i > 1; --i)
            {
                *value = dot2D(v, body.vertices[i]);
                if (*value < best)
                {
                    return best_index;
                }
                else if (*value > best)
                {
                    best = *value;
                    best_index = i;
                }
                else
                {
                    continue;
                }
            }
            return best_index;
        }
        next_id++;
        last_id--;
    }
    return 0;
}

/** hill climb support */
inline size_t Support2D(const ShapeProxy2D &body, const Vector2r &v)
{
    real last, value, next;
    size_t best_index = 0;
    size_t size = body.size;
    if (size >= 3)
    {
        /** determine climb direction */
        next = dot2D(v, body.vertices[1]);
        value = dot2D(v, body.vertices[0]);
        last = dot2D(v, body.vertices[size - 1]);
        real best;
        if ((value > next && value > last) || (value == next && value > last) ||
            (value > next && value == last))
        {
            return 0;
        }

        else if ((value == next) && (value == last))
        {
            return getClimbDirection(body, v, &value);
        }
        else if ((next > value && next >= last) ||
                 ((next == value) && next > last))
        {
            /** ccw search direction,  last <= value */
            if (size == 3) return 1;
            best = next;
            best_index = 1;

            for (size_t i = 2; i < size; ++i)
            {
                value = dot2D(v, body.vertices[i]);
                if (value < best)
                {
                    return best_index;
                }
                else if (value > best)
                {
                    best = value;
                    best_index = i;
                }
                else
                {
                    continue;
                }
            }  // for loop
            return best_index;
        }
        else
        {
            /** cw search direction, next <= value, if local maximum exists,
             * will return the greater index */
            if (size == 3) return 2;
            best = last;
            best_index = size - 1;
            for (size_t i = size - 2; i > 1; --i)
            {
                value = dot2D(v, body.vertices[i]);
                if (value < best)
                {
                    return best_index;
                }
                else if (value > best)
                {
                    best = value;
                    best_index = i;
                }
                else
                {
                    continue;
                }
            }
            return best_index;
        }
    }
    else if (size == 2)
    {
        last = dot2D(v, body.vertices[0]);
        next = dot2D(v, body.vertices[1]);
        return (next > last ? 1 : 0);
    }
    else
    {
        return 0;
    }
    return 0;
}

class GJK2D
{
public:
    static constexpr std::size_t max_iter = 25;
    static constexpr real eps_rel = ESP_REL;

    /** tolarence */
    static constexpr real eps_tol = 1e-10;
    /** zero */
    static constexpr real eps_zero = 1e-5;
    /** rel^2 */
    static constexpr real eps_rel2 = eps_rel * eps_rel;
    /** local minimum1: add a deflection for the search direction */
    static constexpr real eps_deflection = 1e-3;

    Simplex2D simplex_;

    inline uint8_t SameSign(const real &a, const real &b)
    {
        return ((a > 0) == (b > 0));
    };

    /**
     * \brief There are two local minimum problems in the gjk algorithm.
     * The first one:
     * In a given search direction, there are two support points in a convex
     * hull.  One corner case is as follows.
     *
     *                     *-----------*
     *                   *             |
     *   --->          *               |
     *                   *             |
     *                     *-----------*
     * \param[in] body
     * \param[in] v The start search direction. If exsits localminimum1,
     * then return True.
     * \return true If exists localminimum1
     * \return false If not exsits localminimum1
     */
    bool localMinimum1(const ShapeProxy2D &body, const Vector2r &v)
    {
        /** to judge the first point is wether a support point or not */
        bool local_minimum_fake = false;
        int32_t better = 0;
        std::size_t i;
        real max0 = dot2D(body.vertices[0], v);
        real s;

        for (i = 1; i < body.size; ++i)
        {
            s = dot2D(body.vertices[i], v);
            if (s > max0)
            {
                max0 = s;
                better = i;
            }
            else if (s == max0)
            {
                if (better != 0)
                {
                    return true;
                }
                local_minimum_fake = true;
                continue;
            }
            else
            {
                continue;
            }
        }

        if (better == 0 && local_minimum_fake) return true;
        return false;
    }

    /**
     * \brief The second localminimum.
     * There are four support points are collinear for two convex shapes,
     * which means the line segement linked by two vertices in the simplex
     * contains the origin.One corner case is as follows:
     *                       *------------------------- *
     *        *--------*   *                              *
     *      *            *                                  *
     *    *            *   *                                  *
     *  *            *       *                              *
     *    *            *    *                             *
     *      *             *                             *
     *        *---------*    *------------------------*
     * \param[in] P
     * \param[in] Q
     * \param[in\out] v The first vertex in the simplex, if exsits
     * localminimum2, then change v.
     * \return true
     * \return false
     */
    bool localMinimum2(const ShapeProxy2D &P, const ShapeProxy2D &Q,
                       Vector2r &v)
    {
        std::size_t p_idx = Support2D(P, -v);
        std::size_t q_idx = Support2D(Q, v);
        Vector2r next_support_p = P.vertices[p_idx] - Q.vertices[q_idx];

        /** judge the edge linked by the two support points whether contains the
            origin. */
        if (dot2D(v, next_support_p) < 0.0 &&
            apollo_fequal(cross2D(v, next_support_p), 0.0))
        {
            v = ccwNormal2D(v - next_support_p);
            return true;
        }
        return false;
    }

    /**
     * \brief This is the local minimum API for localminimum1 and
     * localminimum2, Distance API will call the localminimum API.
     *
     * \param[in] P
     *
     * \param[in] Q
     * \param[in\out] v The input v is the start search direction, output v
     * is next search direction.
     */
    void localMinimum(const ShapeProxy2D &P, const ShapeProxy2D &Q,
                      Vector2r & v)
    {
        std::size_t vp_idx, vq_idx;
        /** local minimum1: add deflection */
        if (localMinimum1(P, v) && localMinimum1(Q, -v))
        {
            v += ccwNormal2D(v) * eps_deflection;

            vp_idx = Support2D(P, v);
            vq_idx = Support2D(Q, -v);
            v = P.vertices[vp_idx] - Q.vertices[vq_idx];
        }

        vp_idx = Support2D(P, v);
        vq_idx = Support2D(Q, -v);
        v = P.vertices[vp_idx] - Q.vertices[vq_idx];

        simplex_.num = 1;
        simplex_.vertices.row(0) = v;
        simplex_.pwids(0) = vp_idx;
        simplex_.qwids(0) = vq_idx;
        simplex_.lambdas(0) = 1.0;

        /** local minimum 2: */
        localMinimum2(P, Q, v);
    }

    /**
     * \brief Get the Point Contact Directions. The first direction
     * (DisResult2D.normal) is the lower bound of the seperated direction
     * region,　and the second direction (DisResult2D.second_dir) is the upper
     * bound of　the seperated direction region.
     * \param[in] P
     * \param[in] Q
     * \param[out] result
     */
    void getPointContactDirections(const ShapeProxy2D &P, const ShapeProxy2D &Q,
                                   DistResult2D *result)
    {
        switch (simplex_.num)
        {
            case 0:
                break;
            case 2:
            {
                /** p vector */
                if (simplex_.pwids[0] != simplex_.pwids[1])
                {
                    std::size_t id0 = simplex_.pwids[0];
                    std::size_t id1 = simplex_.pwids[1];
                    size_t p_max_id = P.size - 1;

                    if (id0 == 0 && id1 == p_max_id)
                    {
                        result->normal = P.vertices[id0] - result->p;
                        result->second_direction = P.vertices[id1] - result->p;
                    }

                    else if (id1 == 0 && id0 == p_max_id)
                    {
                        result->normal = P.vertices[id1] - result->p;
                        result->second_direction = P.vertices[id0] - result->p;
                    }

                    else
                    {
                        if (id0 < id1)
                        {
                            result->normal = P.vertices[id0] - result->p;
                            result->second_direction =
                                    P.vertices[id1] - result->p;
                        }
                        else
                        {
                            result->normal = P.vertices[id1] - result->p;
                            result->second_direction =
                                    P.vertices[id0] - result->p;
                        }
                    }
                }
                else
                {
                    std::size_t id0 = simplex_.qwids[0];
                    std::size_t id1 = simplex_.qwids[1];
                    size_t q_max_id = Q.size - 1;

                    if (id0 == 0 && id1 == q_max_id)
                    {
                        result->normal = Q.vertices[id0] - result->q;
                        result->second_direction = Q.vertices[id1] - result->q;
                    }

                    else if (id1 == 0 && id0 == q_max_id)
                    {
                        result->normal = Q.vertices[id1] - result->q;
                        result->second_direction = Q.vertices[id0] - result->q;
                    }

                    else
                    {
                        if (id0 < id1)
                        {
                            result->normal = Q.vertices[id0] - result->q;
                            result->second_direction =
                                    Q.vertices[id1] - result->q;
                        }
                        else
                        {
                            result->normal = Q.vertices[id1] - result->q;
                            result->second_direction =
                                    Q.vertices[id0] - result->q;
                        }
                    }
                }

                break;
            }
            case 1:
            case 3:
            {
                /** p vector */
                Vector2r p_cur = P.vertices[simplex_.pwids[0]];

                int last_p_id = simplex_.pwids[0] - 1;
                if (last_p_id == -1)
                {
                    last_p_id = P.size - 1;
                }
                std::size_t next_p_id = simplex_.pwids[0] + 1;
                if (next_p_id == P.size)
                {
                    next_p_id = 0;
                }
                Vector2r p_last = P.vertices[last_p_id];
                Vector2r p_next = P.vertices[next_p_id];

                Vector2r last_cur_vector_p = p_cur - p_last;
                Vector2r next_cur_vector_p = p_cur - p_next;

                /** q vector */
                Vector2r q_cur = Q.vertices[simplex_.qwids[0]];
                int last_q_id = simplex_.qwids[0] - 1;
                if (last_q_id == -1)
                {
                    last_q_id = Q.size - 1;
                }
                std::size_t next_q_id = simplex_.qwids[0] + 1;
                if (next_q_id == Q.size)
                {
                    next_q_id = 0;
                }
                Vector2r q_last = Q.vertices[last_q_id];
                Vector2r q_next = Q.vertices[next_q_id];

                Vector2r cur_last_vector_q = q_last - q_cur;
                Vector2r cur_next_vector_q = q_next - q_cur;

                real p_lower =
                        atan2(last_cur_vector_p(1), last_cur_vector_p(0));
                real p_upper =
                        atan2(next_cur_vector_p(1), next_cur_vector_p(0));

                if (p_upper < p_lower)
                {
                    p_upper = p_upper + cdl::constants::pi() * 2.0;
                }

                real q_lower =
                        atan2(cur_last_vector_q(1), cur_last_vector_q(0));
                real q_upper =
                        atan2(cur_next_vector_q(1), cur_next_vector_q(0));

                if (q_upper < q_lower)
                {
                    q_upper = q_upper + cdl::constants::pi() * 2.0;
                }

                real upper, lower;
                if (q_upper < p_upper)
                {
                    upper = q_upper;
                }
                else
                {
                    upper = p_upper;
                }

                if (q_lower < p_lower)
                {
                    lower = p_lower;
                }
                else
                {
                    lower = q_lower;
                }

                result->second_direction(0) = cos(upper);
                result->second_direction(1) = sin(upper);

                result->normal(0) = cos(lower);
                result->normal(1) = sin(lower);
                break;
            }

            default:
                break;
        }
    }

    void S1D2d(Simplex2D &simplex, Vector2r &v)
    {
        Vector2r a, b, e_ab;
        b = simplex.vertices.row(0);
        a = simplex.vertices.row(1);

        e_ab = b - a;

        /** a region */
        real d_ab_a = -dot2D(a, e_ab);
        if (d_ab_a <= 0.0f)
        {
            simplex.num = 1;
            simplex.vertices.row(0) = simplex.vertices.row(1);
            simplex.lambdas(0) = 1.0;
            simplex.pwids(0) = simplex.pwids(1);
            simplex.qwids(0) = simplex.qwids(1);
            v = a;
            return;
        }
        /** b region */
        real d_ab_b = dot2D(b, e_ab);
        if (d_ab_b <= 0.0f)
        {
            v = b;
            simplex.num = 1;
            simplex.lambdas(0) = 1.0;
            return;
        }

        real inv_d_ab = 1.0 / (d_ab_a + d_ab_b);
        simplex.lambdas(0) = d_ab_a * inv_d_ab;      /**< b_lambda */
        simplex.lambdas(1) = 1 - simplex.lambdas(0); /**< a_lambda */
        v = simplex.lambdas(0) * simplex.vertices.row(0) +
            simplex.lambdas(1) * simplex.vertices.row(1);

        simplex.num = 2;
    };  // GJK2dS1D

    enum struct VertexRegion
    {
        C_VERTEX,
        B_VERTEX,
        A_VERTEX
    };

    /**
     * \brief When origion point is in one of the vertex region of
     * the second simplex.
     *
     * \param vertex: vertex region,
     * \param left : left vertex of the current vertex.
     * \param right  right vertex of the current vertex
     * \param region
     * \param v
     **/
    void S2D2dVertexRegion(const Vector2r &vertex, const Vector2r &left,
                           const Vector2r &right, const VertexRegion &region,
                           Vector2r &v)
    {
        Vector2r left_edge = vertex - left;
        Vector2r right_edge = vertex - right;

        if (dot2D(left_edge, right_edge) < 0.0)
        {
            real v_left_e, left_e, inv_e;
            /** determine the voronoi region */
            v_left_e = dot2D(vertex, left_edge);
            if (v_left_e > 0.0)
            {
                /** c-ac edge, b-ab edge, or a-ba edge */
                left_e = -dot2D(left, left_edge);
                inv_e = 1.0 / (v_left_e + left_e);
                switch (region)
                {
                    case VertexRegion::C_VERTEX:
                        simplex_.lambdas(0) = left_e * inv_e;
                        simplex_.pwids(1) = simplex_.pwids(2);
                        simplex_.qwids(1) = simplex_.qwids(2);
                        simplex_.vertices.row(1) = simplex_.vertices.row(2);
                        break;
                    case VertexRegion::B_VERTEX:
                        simplex_.lambdas(0) = left_e * inv_e;
                        simplex_.vertices.row(0) = simplex_.vertices.row(2);
                        simplex_.qwids(0) = simplex_.qwids(2);
                        simplex_.pwids(0) = simplex_.pwids(2);
                        break;
                    case VertexRegion::A_VERTEX:
                        simplex_.lambdas(0) = left_e * inv_e;
                        simplex_.vertices.row(0) = simplex_.vertices.row(2);
                        simplex_.qwids(0) = simplex_.qwids(2);
                        simplex_.pwids(0) = simplex_.pwids(2);
                        break;
                }
                simplex_.lambdas(1) = 1 - simplex_.lambdas(0);
                simplex_.num = 2;
                v = simplex_.lambdas(0) * simplex_.vertices.row(0) +
                    simplex_.lambdas(1) * simplex_.vertices.row(1);
                return;
            }

            real v_right_e, right_e;
            v_right_e = dot2D(vertex, right_edge);
            if (v_right_e > 0.0)
            {
                /** c-BC edge, b-cb edge, a-ca edge */
                right_e = -dot2D(right, right_edge);
                inv_e = 1.0 / (v_right_e + right_e);
                switch (region)
                {
                    case VertexRegion::C_VERTEX:
                        simplex_.lambdas(0) = right_e * inv_e;
                        break;
                    case VertexRegion::B_VERTEX:
                        simplex_.lambdas(0) = right_e * inv_e;
                        break;
                    case VertexRegion::A_VERTEX:
                        simplex_.lambdas(0) = v_right_e * inv_e;
                        simplex_.pwids(1) = simplex_.pwids(2);
                        simplex_.qwids(1) = simplex_.qwids(2);
                        simplex_.vertices.row(1) = simplex_.vertices.row(2);
                        break;
                }
                simplex_.lambdas(1) = 1 - simplex_.lambdas(0);
                simplex_.num = 2;
                v = simplex_.lambdas(0) * simplex_.vertices.row(0) +
                    simplex_.lambdas(1) * simplex_.vertices.row(1);
                return;
            }
        }

        /** if the mapping point of the origin is not in an edge,
         *  then we only keeps one vertex. */
        simplex_.num = 1;
        simplex_.lambdas(0) = 1.0;
        simplex_.vertices.row(0) = vertex;
        v = vertex;
        switch (region)
        {
            case VertexRegion::C_VERTEX:
                break;
            case VertexRegion::B_VERTEX:
                /** B vertex -+- */
                simplex_.qwids(0) = simplex_.qwids(1);
                simplex_.pwids(0) = simplex_.pwids(1);
                break;
            case VertexRegion::A_VERTEX:
                /** A region +-- */
                simplex_.qwids(0) = simplex_.qwids(2);
                simplex_.pwids(0) = simplex_.pwids(2);
                break;
        }
        return;
    };

    void S2D2d(Simplex2D &simplex, Vector2r &v)
    {
        uint8_t barycentric_codes;
        Vector2r a, b, c;
        a = simplex.vertices.row(2);
        b = simplex.vertices.row(1);
        c = simplex.vertices.row(0);

        Vector3r barycentric;
        barycentric(0) = b(0) * c(1) - c(0) * b(1); /**< w */
        barycentric(1) = c(0) * a(1) - a(0) * c(1); /**< v */
        barycentric(2) = a(0) * b(1) - b(0) * a(1); /**< u */

        real denom = barycentric(0) + barycentric(1) + barycentric(2);
        barycentric_codes = SameSign(denom, barycentric(2));
        barycentric_codes ^= (SameSign(denom, barycentric(1)) << 1);
        barycentric_codes ^= (SameSign(denom, barycentric(0)) << 2);

        switch (barycentric_codes)
        {
            case 4:
                /** A region +-- */
                S2D2dVertexRegion(a, b, c, VertexRegion::A_VERTEX, v);
                break;
            case 5:
                /** AC region +-+ */
                simplex.num = 2;
                simplex.vertices.row(1) = simplex.vertices.row(2);
                simplex.pwids(1) = simplex.pwids(2);
                simplex.qwids(1) = simplex.qwids(2);
                S1D2d(simplex, v);
                break;
            case 6:
                /** AB region ++- */
                simplex.num = 2;
                simplex.vertices.row(0) = simplex.vertices.row(2);
                simplex.pwids(0) = simplex.pwids(2);
                simplex.qwids(0) = simplex.qwids(2);
                S1D2d(simplex, v);
                break;
            case 7:
                /** ABC region +++ */
                real inv_abc = 1.0 / denom;
                simplex.lambdas(0) = barycentric(2) * inv_abc; /**< c */
                simplex.lambdas(1) = barycentric(1) * inv_abc; /**< b */
                simplex.lambdas(2) =
                        1 - simplex.lambdas(0) - simplex.lambdas(1); /**< a */
                simplex.num = 3;
                for (std::size_t j = 0; j < 2; ++j)
                {
                    v(j) = 0;

                    for (std::size_t i = 0; i < simplex.num; ++i)
                    {
                        v(j) += simplex.lambdas(i) * simplex.vertices(i, j);
                    }
                }
                break;
        }
    };

    void SubDistance(Simplex2D &simplex, Vector2r &v)
    {
        switch (simplex.num)
        {
            case 3:
                S2D2d(simplex, v);
                break;
            case 2:
                S1D2d(simplex, v);
                break;
            case 1:
                simplex_.lambdas(0) = 1.0;
                v = simplex_.vertices.row(0);
        }
    };

    /**
     * \brief This distance API is designed for the EPA2D algorithm. If two
     * objects are penetrated, then this api will return the complete simplex.
     * The API can resolve the degenerate cases.
     *
     * \param[in] P Convex shape P
     * \param[in] Q Convex shape Q
     * \param[out] result The result information
     * \return real The distance information is returned.
     **/
    real Distance(const ShapeProxy2D &P, const ShapeProxy2D &Q,
                  DistResult2D *result);

    /**
     * \brief Barycode-based distance implementation without degenerated cases
     * handling. This API may generate incomplete simplex when return.
     * If you just want to get the positive distance, this distance
     * API is the fastest version compared with distance and distance3 API.
     * The only difference of the two distance implementaiton (Distance2
     * and Distance3) is how to generate the first simplex. In this API,
     * The start support direction is generated by the Minkowski difference
     * of two arbitrary vertices of the shapes. So the first simplex vertex
     * maybe not located in the boundary of the CSO.
     *
     * \param P
     * \param Q
     * \param result
     * \return real
     */
    real Distance2(const ShapeProxy2D &P, const ShapeProxy2D &Q,
                   DistResult2D *result);

    /**
     * \brief Barycode-based distance implementation without degenerated cases
     * handling. If two objects are penetrated, this API may generate incomplete
     * simplex when return. The start support direction is given by a fixed
     * support direction, so the first simplex vertex lies in the boundary of
     * the CSO. And EPA will not fall into a local stationary point because of
     * this performance brought by this start way.
     *
     * \param[in] P
     * \param[in] Q
     * \param[out] result
     * \return real
     */
    real Distance3(const ShapeProxy2D &P, const ShapeProxy2D &Q,
                   DistResult2D *result);

    /** to get colision information: return true if collision, return false
     * if no collision. */
    bool Collision(const ShapeProxy2D &P, const ShapeProxy2D &Q);

    void setShapeEdgeNormal(Vector2r *vertices, uint32_t &size,
                            Vector2r normals[]);

    /**
     * \brief
     * reference:
     * [1] Ray Casting against General Convex Objects with Application to
     * Continuous Collision Detection.
     * [2] "Smooth Mesh Contacts with GJK" in the book(Game Physics Pearls.2010)
     * [3] http://www.dtecta.com/files/GDC2011_vandenBergen_Gino_Physics_Tut.pdf
     * \param input
     * \param output
     */
    bool traversalRayCast(RayCastRequest *input, RayCastResult *output);

    /**
     * \brief different form traversalRayCast method, the gjk based ray cast
     * using gjk algorithm
     * to find a closest point to iteratively solve the hit spot. the algorithm3
     * in [1]
     *
     * \param[in] input
     * \param[out] output
     * \return true If hit.
     * \return false If no hit.
     */
    bool GJKBasedRayCast(RayCastRequest *input, RayCastResult *output);

    /**
     * \brief Shape cast algorithm
     * Reference: "Smooth Mesh Contacts with GJK" in the book(Game Physics
     * Pearls.2010) Attention: in the ShapeCastRequest, translation = proxy_end
     * - proxy_start, and the value of translation is no need normalized. And in
     * the algorithm, the value of time is [0, 1]. For proxy A, the moving
     * distance that hits with a polygon is d = time * translationA.norm().
     * \param[in] input
     * \param[out] output
     * \return true If hit.
     * \return false If no hit.
     */
    bool shapeCast(ShapeCastResult *output, ShapeCastRequest *input);

};  // class GJK2d

}  // namespace np2d
}  // namespace cdl