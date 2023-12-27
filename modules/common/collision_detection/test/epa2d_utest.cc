/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include <gtest/gtest.h>
#include <iostream>
#include "constants.h"
#include "cdl_math.h"
#include "types.h"
#include "gjk2d.h"
#include "epa2d.h"
#include "modules/common/math/pose.h"

using namespace cdl;
using namespace cdl::np2d;
using namespace apollo;

TEST(test_collision_detection_epa2d, test_calculateClosestEdge)
{
    /* Get the index of the minimum of distance_to_origin in the array */
    EPA2D epa2d;
    size_t closest_edge_idx;//Both input and output

    epa2d.vertice_num = 3;
    epa2d.epa_edges_[0].distance_to_origin = 1.5;
    epa2d.epa_edges_[1].distance_to_origin = 1.15;
    epa2d.epa_edges_[2].distance_to_origin = 1.7;

    epa2d.calculateClosestEdge(&closest_edge_idx);
    EXPECT_EQ(closest_edge_idx, 1);
}

TEST(test_collision_detection_epa2d, test_originToEdgeDistance)
{
    /* Distance from origin to straight line */
    Vector2r point1, point2;
    EPA2D epa2d;
    real ret;

    /* The origin is on both sides of point 1 and point 2 */
    point1 = {1.0, 1.5};
    point2 = {2.0, 3.5};
    ret = epa2d.originToEdgeDistance(point1, point2);
    EXPECT_FLOAT_EQ(ret, 1.0/apollo_sqrt(5)/2.0);

    point1 = {-2.0, 0.0};
    point2 = {-1.0, 1.0};
    ret = epa2d.originToEdgeDistance(point1, point2);
    EXPECT_FLOAT_EQ(ret, -apollo_sqrt(2.0));

    /* The origin coincides with point 1 and point 2 */
    point1 = {0.0, 0.0};
    point2 = {0.0, 0.0};
    ret = epa2d.originToEdgeDistance(point1, point2);
    EXPECT_FLOAT_EQ(ret, 0.0);

    /* The origin is between point 1 and point 2 */
    point1 = {-2.0, 0.0};
    point2 = {0.0, 2.0};
    ret = epa2d.originToEdgeDistance(point1, point2);
    EXPECT_FLOAT_EQ(ret, -apollo_sqrt(2.0));
}

TEST(test_collision_detection_epa2d, test_enableCounterClockwise)
{
    /* Enable the simplex counter-clockwise and
     * record the distance from the origin to each side of the simplex. */
    ShapeProxy2D  P, Q;
    DistResult2D gjk2d_result;
    EPA2D epa2d;
    GJK2D gjk2d;

    Position2D P_pos_case1[6] = {
        {3, 0},
        {4, -2},
        {5, -2},
        {6, 0},
        {5, 2},
        {4, 2}};

    Position2D Q_pos_case1[6] = {
        {1, 0},
        {2, -2},
        {3, -2},
        {4, 0},
        {3, 2},
        {2, 2}};

    P.size = 6;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case1[i].x;
        P.vertices[i](1) = P_pos_case1[i].y;
    }

    Q.size = 6;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case1[i].x;
        Q.vertices[i](1) = Q_pos_case1[i].y;
    }
    gjk2d.Distance(P, Q, &gjk2d_result);
    EXPECT_FLOAT_EQ(gjk2d_result.distance, 0.0);
    EXPECT_EQ(gjk2d.simplex_.num, 3);

    /* Record the coordinates of the simplex(clockwise) */
    EXPECT_FLOAT_EQ(gjk2d.simplex_.vertices.row(0)(0), 1.0);
    EXPECT_FLOAT_EQ(gjk2d.simplex_.vertices.row(0)(1), 4.0);
    EXPECT_FLOAT_EQ(gjk2d.simplex_.vertices.row(1)(0), 2.0);
    EXPECT_FLOAT_EQ(gjk2d.simplex_.vertices.row(1)(1), -4.0);
    EXPECT_FLOAT_EQ(gjk2d.simplex_.vertices.row(2)(0), -1.0);
    EXPECT_FLOAT_EQ(gjk2d.simplex_.vertices.row(2)(1), 0.0);

    epa2d.enableCounterClockwise(gjk2d.simplex_);

    /* Verify the distance from the simplex edge to the origin */
    EXPECT_FLOAT_EQ(epa2d.epa_edges_[0].distance_to_origin, 2.0/apollo_sqrt(5));
    EXPECT_FLOAT_EQ(epa2d.epa_edges_[1].distance_to_origin, 12.0/apollo_sqrt(65));
    EXPECT_FLOAT_EQ(epa2d.epa_edges_[2].distance_to_origin, 4.0/5.0);

    /* Record the coordinates of the original polygon corresponding
     * to the simplex.
     * */
    EXPECT_EQ(gjk2d.simplex_.pwids(0), 5);
    EXPECT_EQ(gjk2d.simplex_.qwids(0), 2);

    EXPECT_EQ(gjk2d.simplex_.pwids(1), 1);
    EXPECT_EQ(gjk2d.simplex_.qwids(1), 5);

    EXPECT_EQ(gjk2d.simplex_.pwids(2), 0);
    EXPECT_EQ(gjk2d.simplex_.qwids(2), 3);

    /* Verify the direction of the simplex edges */
    EXPECT_EQ(epa2d.epa_edges_[0].p_id(0), 5);
    EXPECT_EQ(epa2d.epa_edges_[0].q_id(0), 2);
    EXPECT_EQ(epa2d.epa_edges_[0].p_id(1), 0);
    EXPECT_EQ(epa2d.epa_edges_[0].q_id(1), 3);

    EXPECT_EQ(epa2d.epa_edges_[1].p_id(0), 1);
    EXPECT_EQ(epa2d.epa_edges_[1].q_id(0), 5);
    EXPECT_EQ(epa2d.epa_edges_[1].p_id(1), 5);
    EXPECT_EQ(epa2d.epa_edges_[1].q_id(1), 2);

    EXPECT_EQ(epa2d.epa_edges_[2].p_id(0), 0);
    EXPECT_EQ(epa2d.epa_edges_[2].q_id(0), 3);
    EXPECT_EQ(epa2d.epa_edges_[2].p_id(1), 1);
    EXPECT_EQ(epa2d.epa_edges_[2].q_id(1), 5);

    gjk2d.Distance(Q, P, &gjk2d_result);
    EXPECT_NEAR(gjk2d_result.distance, 0.0, 0.002);
    EXPECT_EQ(gjk2d.simplex_.num, 3);
}

TEST(test_collision_detection_epa2d, test_pointToEdgeDistance)
{
    /* Distance from point to line. */
    Vector2r point, edge_p0, edge_p1;
    EPA2D epa2d;
    real ret;

    point = {0, 0};
    edge_p0 = {1.0, 0};
    edge_p1 = {0, 1.0};

    ret = epa2d.pointToEdgeDistance(point, edge_p0, edge_p1);
    EXPECT_FLOAT_EQ(ret, apollo_sqrt(2)/2.0);

    point = {0, -1};
    ret = epa2d.pointToEdgeDistance(point, edge_p0, edge_p1);
    EXPECT_FLOAT_EQ(ret, apollo_sqrt(2));

    point = {0, -2};
    ret = epa2d.pointToEdgeDistance(point, edge_p0, edge_p1);
    EXPECT_FLOAT_EQ(ret, 3.0/apollo_sqrt(2));
}

TEST(test_collision_detection_epa2d, test_zeroOrderSimplexExpanding)
{
    /* gjk2d.simplex_.num always greater than or equal to 2.
     * The zeroOrderSimplexExpanding() function will not be called in the code. */
    ShapeProxy2D  P, Q;
    Simplex2D gjk_simplex;
    DistResult2D epa_result;
    EPA2D epa2d;
    GJK2D gjk2d;

    Position2D P_pos_case1[6] = {
        {-1, 0},
        {4, -2},
        {5, -2},
        {6, 0},
        {5, 2},
        {4, 2}};

    Position2D Q_pos_case1[1] = {
        {0, 0}};

    P.size = 6;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case1[i].x;
        P.vertices[i](1) = P_pos_case1[i].y;
    }

    Q.size = 1;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case1[i].x;
        Q.vertices[i](1) = Q_pos_case1[i].y;
    }
    gjk2d.Distance2(P, Q, &epa_result);
    EXPECT_EQ(gjk2d.simplex_.num, 2);

    epa2d.firstOrderSimplexExpanding(P, Q, gjk2d.simplex_, &epa_result);
    EXPECT_EQ(gjk2d.simplex_.num, 3);
}

TEST(test_collision_detection_epa2d, test_firstOrderSimplexExpanding)
{
    /* gjk2d.simplex_.num = 2 */
    ShapeProxy2D  P, Q;
    Simplex2D gjk_simplex;
    DistResult2D epa_result;
    EPA2D epa2d;
    GJK2D gjk2d;

    Position2D P_pos_case1[6] = {
        {3, 0},
        {4, -2},
        {5, -2},
        {6, 0},
        {5, 2},
        {4, 2}};

    Position2D Q_pos_case1[6] = {
        {1, 0},
        {2, -2},
        {3, -2},
        {4, 0},
        {3, 2},
        {2, 2}};

    P.size = 6;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case1[i].x;
        P.vertices[i](1) = P_pos_case1[i].y;
    }

    Q.size = 6;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case1[i].x;
        Q.vertices[i](1) = Q_pos_case1[i].y;
    }
    gjk2d.Distance3(P, Q, &epa_result);
    EXPECT_EQ(gjk2d.simplex_.num, 2);

    epa2d.firstOrderSimplexExpanding(P, Q, gjk2d.simplex_, &epa_result);
    EXPECT_EQ(gjk2d.simplex_.num, 3);
}

TEST(test_collision_detection_epa2d, test_Penetration)
{
    /* gjk2d.simplex_.num = 3 */
    ShapeProxy2D  P, Q;
    DistResult2D epa_result;
    EPA2D epa2d;
    GJK2D gjk2d;

    Position2D P_pos_case1[6] = {
        {3, 0},
        {4, -2},
        {5, -2},
        {6, 0},
        {5, 2},
        {4, 2}};

    Position2D Q_pos_case1[6] = {
        {1, 0},
        {2, -2},
        {3, -2},
        {4, 0},
        {3, 2},
        {2, 2}};

    P.size = 6;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case1[i].x;
        P.vertices[i](1) = P_pos_case1[i].y;
    }

    Q.size = 6;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case1[i].x;
        Q.vertices[i](1) = Q_pos_case1[i].y;
    }

    gjk2d.Distance(P, Q, &epa_result);
    EXPECT_FLOAT_EQ(epa_result.distance, 0.0);
    EXPECT_EQ(gjk2d.simplex_.num, 3);

    epa2d.Penetration(P, Q, gjk2d.simplex_, epa_result);
    EXPECT_FLOAT_EQ(epa_result.distance, -2.0/apollo_sqrt(5));
}

TEST(test_collision_detection_epa2d, test_Penetration2)
{
    /* gjk2d.simplex_.num >= 2 by using distance3 algorithm. */
    ShapeProxy2D  P, Q;
    Simplex2D gjk_simplex;
    DistResult2D epa_result;
    EPA2D epa2d;
    GJK2D gjk2d;

    Position2D P_pos_case1[6] = {
        {3, 0},
        {4, -2},
        {5, -2},
        {6, 0},
        {5, 2},
        {4, 2}};

    Position2D Q_pos_case1[6] = {
        {1, 0},
        {2, -2},
        {3, -2},
        {4, 0},
        {3, 2},
        {2, 2}};

    P.size = 6;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case1[i].x;
        P.vertices[i](1) = P_pos_case1[i].y;
    }

    Q.size = 6;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case1[i].x;
        Q.vertices[i](1) = Q_pos_case1[i].y;
    }
    gjk2d.Distance3(P, Q, &epa_result);

    /* There is no possibility of simplex.num = 1. */
    EXPECT_EQ(gjk2d.simplex_.num, 2);
    epa2d.Penetration2(P, Q, &gjk2d.simplex_, epa_result);
    EXPECT_FLOAT_EQ(epa_result.distance, -2.0/apollo_sqrt(5));

    /* contain */
    Position2D P_pos_case2[4] = {
        {-1, -1},
        {1, -1},
        {1, 1},
        {-1, 1}};

    Position2D Q_pos_case2[4] = {
        {-2, -2},
        {2, -2},
        {2, 2},
        {-2, 2}};

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case2[i].x;
        P.vertices[i](1) = P_pos_case2[i].y;
    }

    Q.size = 4;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case2[i].x;
        Q.vertices[i](1) = Q_pos_case2[i].y;
    }
    gjk2d.Distance(P, Q, &epa_result);

    EXPECT_EQ(gjk2d.simplex_.num, 3);
    epa2d.Penetration(P, Q, gjk2d.simplex_, epa_result);

    EXPECT_FLOAT_EQ(epa_result.distance, -3.0);

    /* Three points are collinear in the polygon_Q. */
    Position2D P_pos_case3[4] = {
        {-1, -1},
        {1, -1},
        {1, 1},
        {-1, 1}};

    Position2D Q_pos_case3[4] = {
        {-2, -2},
        {2, -2},
        {2, 2},
        {-1, -1}};

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case3[i].x;
        P.vertices[i](1) = P_pos_case3[i].y;
    }

    Q.size = 4;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case3[i].x;
        Q.vertices[i](1) = Q_pos_case3[i].y;
    }
    gjk2d.Distance(P, Q, &epa_result);

    EXPECT_EQ(gjk2d.simplex_.num, 3);
    epa2d.Penetration(P, Q, gjk2d.simplex_, epa_result);

    EXPECT_FLOAT_EQ(epa_result.distance, -apollo_sqrt(2.0));
}