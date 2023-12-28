/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include "cdl.h"
#include "gjk2d_interface.h"

using namespace apollo;

void printShapeCastResult(const cdl::np2d::ShapeCastResult &result)
{
    printf("hit spot A [%.3f, %.3f] \n", result.collision_pointA(0),
           result.collision_pointA(1));
    printf("hit spot B [%.3f, %.3f] \n", result.collision_pointB(0),
           result.collision_pointB(1));
    printf("dis a %.3f \n", result.distanceA);
    printf("dis b %.3f \n", result.distanceB);
    printf("iter %d \n", int(result.iterations));
    if (result.collided)
    {
        printf("collision \n");
    }
    else
    {
        printf("no collision\n");
    }
}

int32_t collision_point_is_overlap(const cdl::np2d::ShapeCastRequest &input,
                                   const cdl::np2d::ShapeCastResult &result)
{
    if (!result.collided)
    {
        return 1;
    }
    cdl::Vector2r collision_p, collision_q;
    if (input.translationA == cdl::Vector2r::Zero())
    {
        collision_p = result.collision_pointA;
    }
    else
    {
        collision_p = result.collision_pointA +
                      result.distanceA * input.translationA.normalized();
    }
    if (input.translationB == cdl::Vector2r::Zero())
    {
        collision_q = result.collision_pointB;
    }
    else
    {
        collision_q = result.collision_pointB +
                      result.distanceB * input.translationB.normalized();
    }
    cdl::Vector2r diff = collision_p - collision_q;

    if (cdl::dot2D(diff, diff) < cdl::constants::eps())
    {
        return 1;
    }
    printf("diff %.4f \n", cdl::dot2D(diff, diff));
    return 0;
}

using namespace cdl;
using namespace cdl::np2d;

TEST(shapecast, test1)
{
    cdl::np2d::ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));

    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 0.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 5));

    request.proxyA.size = 4;
    request.proxyB.size = 4;
    request.translationB = Vector2r::Zero();
    request.translationA = Vector2r(10, 0);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test1_1)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));

    request.proxyA.size = 4;
    request.proxyB.size = 4;
    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 0.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 5));

    for (size_t i = 0; i < request.proxyB.size; i++)
    {
        request.proxyB.vertices[i] += Vector2r(-7, -3);
    }

    request.translationB = Vector2r::Zero();
    request.translationA = Vector2r(10, 0);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test1_2)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));

    request.proxyA.size = 4;
    request.proxyB.size = 4;
    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 0.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 5));

    for (size_t i = 0; i < request.proxyB.size; i++)
    {
        request.proxyB.vertices[i] += Vector2r(0, 10);
    }

    request.translationB = Vector2r::Zero();
    request.translationA = Vector2r(10, 0);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test2)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));

    request.proxyB.vertices[0] = (Vector2r(10, 1.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 1.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, 6.0));
    request.proxyB.vertices[3] = (Vector2r(10, 6));

    request.proxyA.size = 4;
    request.proxyB.size = 4;
    request.translationB = Vector2r::Zero();
    request.translationA = Vector2r(5, 0);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test3)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));

    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 0.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 5));

    request.proxyA.size = 4;
    request.proxyB.size = 4;
    request.translationB = Vector2r::Zero();
    request.translationA = Vector2r(5, -5);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test4)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));

    request.proxyB.vertices[0] = (Vector2r(10, -10.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, -10.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, -2.0));
    request.proxyB.vertices[3] = (Vector2r(10, -2));

    request.proxyA.size = 4;
    request.proxyB.size = 4;
    request.translationB = Vector2r::Zero();
    request.translationA = Vector2r(5, -5);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test5)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));

    request.proxyB.vertices[0] = (Vector2r(10, -10.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, -10.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, -2.0));
    request.proxyB.vertices[3] = (Vector2r(10, -2));

    request.proxyA.size = 4;
    request.proxyB.size = 4;
    request.translationB = Vector2r::Zero();
    request.translationA = Vector2r(5, 0);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test6)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));

    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 0.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 5));

    request.proxyA.size = 4;
    request.proxyB.size = 4;
    request.translationB = Vector2r::Zero();
    request.translationA = Vector2r(1, 0);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test7)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));

    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 0.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 5));
    request.proxyB.vertices[4] = (Vector2r(8, 3));

    request.proxyA.size = 4;
    request.proxyB.size = 5;
    request.translationB = Vector2r::Zero();
    request.translationA = Vector2r(6, 0);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test8)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));
    request.proxyA.size = 4;
    request.proxyB.size = 5;

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] += Vector2r(20, 0);
    }

    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 0.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 5));
    request.proxyB.vertices[4] = (Vector2r(8, 3));

    request.translationB = Vector2r::Zero();
    request.translationA = Vector2r(-6, 0);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test9)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));
    request.proxyA.size = 4;
    request.proxyB.size = 5;

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] += Vector2r(20, 0);
    }

    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 0.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 5));
    request.proxyB.vertices[4] = (Vector2r(8, 3));

    request.translationB = Vector2r::Zero();
    request.translationA = Vector2r(-6, -1);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

/** collision */
TEST(shapecast, test16)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(-77754.35727, 4389718.35596));
    request.proxyA.vertices[1] = (Vector2r(-77754.32772, 4389715.89614));
    request.proxyA.vertices[2] = (Vector2r(-77749.70306, 4389715.95170));
    request.proxyA.vertices[3] = (Vector2r(-77749.73261, 4389718.41152));
    request.proxyA.size = 4;
    request.proxyB.size = 4;

    request.proxyB.vertices[0] = (Vector2r(-77756.04101, 4389718.14834));
    request.proxyB.vertices[1] = (Vector2r(-77756.47623, 4389717.24802));
    request.proxyB.vertices[2] = (Vector2r(-77755.57591, 4389716.81280));
    request.proxyB.vertices[3] = (Vector2r(-77755.14069, 4389717.71312));

    request.translationA = Vector2r(0, 0);
    request.translationB = Vector2r(1.4200, 1.8645);

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] -= Vector2r(-77754.35727, 4389718.35596);
    }
    for (size_t i = 0; i < request.proxyB.size; i++)
    {
        request.proxyB.vertices[i] -= Vector2r(-77754.35727, 4389718.35596);
    }

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test14)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(6.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(6.0, 4.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 4.0));
    request.proxyA.size = 4;
    request.proxyB.size = 5;

    cdl::Transform2r obs_start_tf;
    obs_start_tf.setIdentity();
    obs_start_tf.translation() = cdl::Vector2r(0, 0);
    Eigen::Rotation2D<cdl::real> r(45.0);
    obs_start_tf.linear() = r.toRotationMatrix();

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] = obs_start_tf * request.proxyA.vertices[i];
    }

    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 1.0));
    request.proxyB.vertices[2] = (Vector2r(14.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 4));
    request.proxyB.vertices[4] = (Vector2r(8, 3));

    request.translationA = Vector2r(100, 0);
    request.translationB = Vector2r(100, 0);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

/** no collision */
TEST(shapecast, test13)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(6.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(6.0, 4.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 4.0));
    request.proxyA.size = 4;
    request.proxyB.size = 5;

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] += Vector2r(0, 10);
    }

    cdl::Transform2r obs_start_tf;
    obs_start_tf.setIdentity();
    obs_start_tf.translation() = cdl::Vector2r(0, 0);
    Eigen::Rotation2D<cdl::real> r(10.0);
    obs_start_tf.linear() = r.toRotationMatrix();

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] = obs_start_tf * request.proxyA.vertices[i];
    }

    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 1.0));
    request.proxyB.vertices[2] = (Vector2r(14.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 4));
    request.proxyB.vertices[4] = (Vector2r(8, 3));

    request.translationA = Vector2r(100, 10);
    request.translationB = Vector2r(60, 100);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test12)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(6.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(6.0, 4.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 4.0));
    request.proxyA.size = 4;
    request.proxyB.size = 5;

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] += Vector2r(0, 10);
    }

    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 1.0));
    request.proxyB.vertices[2] = (Vector2r(14.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 4));
    request.proxyB.vertices[4] = (Vector2r(8, 3));

    request.translationA = Vector2r(100, 0);
    request.translationB = Vector2r(60, 100);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test11)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 1.0));
    request.proxyA.vertices[2] = (Vector2r(3.0, 5.0));
    request.proxyA.size = 3;
    request.proxyB.size = 5;

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] += Vector2r(0, 10);
    }

    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 1.0));
    request.proxyB.vertices[2] = (Vector2r(14.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 4));
    request.proxyB.vertices[4] = (Vector2r(8, 3));

    request.translationA = Vector2r(1, -10);
    request.translationB = Vector2r(-10, 1);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test1_frame69)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(-77175.4471, 4389734.0137));
    request.proxyA.vertices[1] = (Vector2r(-77173.2807, 4389732.8483));
    request.proxyA.vertices[2] = (Vector2r(-77171.0898, 4389736.9215));
    request.proxyA.vertices[3] = (Vector2r(-77173.2562, 4389738.0868));

    request.proxyB.vertices[0] = (Vector2r(-77174.8366, 4389740.3092));
    request.proxyB.vertices[1] = (Vector2r(-77175.1801, 4389738.3097));
    request.proxyB.vertices[2] = (Vector2r(-77174.2001, 4389736.8835));
    request.proxyB.vertices[3] = (Vector2r(-77173.2907, 4389739.7705));

    request.proxyB.vertices[4] = (Vector2r(-77173.3289, 4389740.0967));
    request.proxyB.vertices[5] = (Vector2r(-77173.5874, 4389740.3224));
    request.proxyB.vertices[6] = (Vector2r(-77173.8697, 4389740.4744));
    request.proxyB.vertices[7] = (Vector2r(-77174.6307, 4389740.6719));

    request.proxyA.size = 4;
    request.proxyB.size = 8;

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] -= Vector2r(-77175.4471, 4389734.0137);
    }
    for (size_t i = 0; i < request.proxyB.size; i++)
    {
        request.proxyB.vertices[i] -= Vector2r(-77175.4471, 4389734.0137);
    }

    request.translationA = Vector2r(-0.651195, -1.00595);
    request.translationB = Vector2r(-1.05274, -2.35429);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test1_bug1)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(-77172.28441, 4389746.19998));
    request.proxyA.vertices[1] = (Vector2r(-77170.48903, 4389745.31873));
    request.proxyA.vertices[2] = (Vector2r(-77169.40950, 4389747.51807));
    request.proxyA.vertices[3] = (Vector2r(-77171.20488, 4389748.39932));

    request.proxyB.vertices[0] = (Vector2r(-77172.20952, 4389742.29975));
    request.proxyB.vertices[1] = (Vector2r(-77172.24642, 4389742.15986));
    request.proxyB.vertices[2] = (Vector2r(-77172.16024, 4389741.78845));
    request.proxyB.vertices[3] = (Vector2r(-77171.00772, 4389741.24185));

    request.proxyB.vertices[4] = (Vector2r(-77170.95542, 4389741.23049));
    request.proxyB.vertices[5] = (Vector2r(-77170.17255, 4389743.50776));
    request.proxyB.vertices[6] = (Vector2r(-77169.98021, 4389744.74348));
    request.proxyB.vertices[7] = (Vector2r(-77170.29947, 4389744.98588));

    request.proxyB.vertices[8] = (Vector2r(-77170.58676, 4389744.85652));
    request.proxyB.vertices[9] = (Vector2r(-77171.02737, 4389744.33404));

    request.proxyA.size = 4;
    request.proxyB.size = 10;

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] += Vector2r(77169.0, -4389769.0);
    }
    for (size_t i = 0; i < request.proxyB.size; i++)
    {
        request.proxyB.vertices[i] += Vector2r(77169.0, -4389769.0);
    }

    request.translationA = Vector2r(-0.286421, -0.527542);
    request.translationB = Vector2r(-0.366914, 0.0416029);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test1_bug0)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(-77169.43898, 4389768.81062));
    request.proxyA.vertices[1] = (Vector2r(-77170.63953, 4389767.21104));
    request.proxyA.vertices[2] = (Vector2r(-77168.68005, 4389765.74036));
    request.proxyA.vertices[3] = (Vector2r(-77167.47949, 4389767.33994));

    request.proxyB.vertices[0] = (Vector2r(-77169.85983, 4389769.44017));
    request.proxyB.vertices[1] = (Vector2r(-77170.02024, 4389769.17887));
    request.proxyB.vertices[2] = (Vector2r(-77169.31755, 4389768.87700));
    request.proxyB.vertices[3] = (Vector2r(-77169.48908, 4389769.31265));

    request.proxyA.size = 4;
    request.proxyB.size = 4;

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] += Vector2r(77169.0, -4389769.0);
    }
    for (size_t i = 0; i < request.proxyB.size; i++)
    {
        request.proxyB.vertices[i] += Vector2r(77169.0, -4389769.0);
    }

    request.translationA = Vector2r(-0.482642, 0.356464);
    request.translationB = Vector2r(0, 0);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast, test10)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));
    request.proxyA.size = 4;
    request.proxyB.size = 5;

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] += Vector2r(0, 10);
    }

    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 0.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 5));
    request.proxyB.vertices[4] = (Vector2r(8, 3));

    request.translationA = Vector2r(0, -10);
    request.translationB = Vector2r(-10, 0);

    cdl::np2d::GJK2D gjk;
    ShapeCastResult result;

    gjk.shapeCast(&result, &request);
    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

TEST(shapecast_with_dir, test1)
{
    ShapeCastRequest request;

    request.proxyA.vertices[0] = (Vector2r(0.0, 0.0));
    request.proxyA.vertices[1] = (Vector2r(5.0, 0.0));
    request.proxyA.vertices[2] = (Vector2r(5.0, 5.0));
    request.proxyA.vertices[3] = (Vector2r(0.0, 5.0));
    request.proxyA.size = 4;
    request.proxyB.size = 5;

    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        request.proxyA.vertices[i] += Vector2r(0, 10);
    }

    request.proxyB.vertices[0] = (Vector2r(10, 0.0));
    request.proxyB.vertices[1] = (Vector2r(15.0, 0.0));
    request.proxyB.vertices[2] = (Vector2r(15.0, 5.0));
    request.proxyB.vertices[3] = (Vector2r(10, 5));
    request.proxyB.vertices[4] = (Vector2r(8, 3));

    request.translationA = Vector2r(0, -10);
    request.translationB = Vector2r(-10, 0);

    ShapeCastResult result;

    bool is_collision;
    doubledist_a, dist_b;
    Position2D pos_a, pos_b, dir_a, dir_b;
    Polygon2D poly_a, poly_b;

    poly_a.vertex_num = request.proxyA.size;
    for (size_t i = 0; i < request.proxyA.size; i++)
    {
        poly_a.vertexes[i].x = request.proxyA.vertices[i](0);
        poly_a.vertexes[i].y = request.proxyA.vertices[i](1);
    }
    poly_b.vertex_num = request.proxyB.size;
    for (size_t i = 0; i < request.proxyB.size; i++)
    {
        poly_b.vertexes[i].x = request.proxyB.vertices[i](0);
        poly_b.vertexes[i].y = request.proxyB.vertices[i](1);
    }
    dir_a.x = request.translationA(0);
    dir_a.y = request.translationA(1);
    dir_b.x = request.translationB(0);
    dir_b.y = request.translationB(1);

    shapecast_with_direction(&is_collision, &dist_a, &dist_b, &pos_a, &pos_b,
            &poly_a, &dir_a, &poly_b, &dir_b);

    if (is_collision)
    {
        result.collided = true;
    }
    else
    {
        result.collided = false;
    }
    result.collision_pointA(0) = pos_a.x;
    result.collision_pointA(1) = pos_a.y;
    result.collision_pointB(0) = pos_b.x;
    result.collision_pointB(1) = pos_b.y;
    result.distanceA = dist_a;
    result.distanceB = dist_b;

    EXPECT_EQ(1, collision_point_is_overlap(request, result));
}

#if 0
TEST(shape_case_test_viz2d, viz2d_start)
{
    return_t ret;
    viz2d_image image_handle;
    viz2d_font_setting font;
    int32_t win_columns;
    int32_t win_rows;
    int32_t origin_columns_index;
    int32_t origin_rows_index;
    doubleresolution;
    viz2d_color background_color;
    char_t win_name[MAX_WINDOW_NAME_LEN];
    CvScalar ele;

    ret = viz2d_set_default_font(&font);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(font.font_type, viz2d_font_hershey_simplex);

    strncpy(win_name, "Debug shapecast", MAX_WINDOW_NAME_LEN);
    win_columns = 1200;
    win_rows = 800;
    origin_columns_index = 200;
    origin_rows_index = 200;
    background_color = viz2d_colors_white;

    resolution = 0.1;
    viz2d_init_image_handle(&image_handle, &font, win_name, win_columns,
            win_rows, origin_columns_index, origin_rows_index, resolution,
            background_color);
    ret = viz2d_start(&image_handle);
    EXPECT_EQ(ret, 0);
    EXPECT_TRUE(NULL != image_handle.image);
    EXPECT_EQ(image_handle.image->nChannels, 3);
    EXPECT_EQ(image_handle.image->depth, 8);
    EXPECT_EQ(image_handle.image->width, win_columns);
    EXPECT_EQ(image_handle.image->height, win_rows);

    ret = viz2d_draw_xy_axis(&image_handle);
    EXPECT_EQ(ret, 0);


    Polygon2D poly_a_start ,poly_a_end;
    Polygon2D poly_b_start, poly_b_end;
#if 1
    Pose2D start_a, end_a, start_b, end_b;
    doublelength, width;
    length = 3.0;
    width = 3.0;

    start_a.pos.x = 1003963.497227;
    start_a.pos.y = 3407264.221959;
    start_a.theta = 0.01;

    end_a.pos.x = 1003973.497098;
    end_a.pos.y = 3407264.272719;
    end_a.theta = 0.01;

    start_b.pos.x = 1003976.054427;
    start_b.pos.y = 3407267.279113;
    start_b.theta = 4.72;

    end_b.pos.x = 1003976.122999;
    end_b.pos.y = 3407257.279348;
    end_b.theta = 4.72;

    ret = generate_rect_polygon_base_1point(
            &poly_a_start, &start_a, length, width);
    EXPECT_EQ(ret, 0);

    ret = generate_rect_polygon_base_1point(&poly_a_end, &end_a, length, width);
    EXPECT_EQ(ret, 0);

    ret = generate_rect_polygon_base_1point(
                &poly_b_start, &start_b, length, width);
    EXPECT_EQ(ret, 0);

    ret = generate_rect_polygon_base_1point(&poly_b_end, &end_b, length, width);
    EXPECT_EQ(ret, 0);
#else
    poly_a_start.vertex_num = 4;
    poly_a_start.vertexes[0].x = 1003973.489484;
    poly_a_start.vertexes[0].y = 3407265.772700;

    poly_a_start.vertexes[1].x = 1003973.504712;
    poly_a_start.vertexes[1].y = 3407262.772738;

    poly_a_start.vertexes[2].x = 1003976.504673;
    poly_a_start.vertexes[2].y = 3407262.787967;

    poly_a_start.vertexes[3].x = 1003976.489445;
    poly_a_start.vertexes[3].y = 3407265.787928;
    poly_a_start.type = POLYGON_RECTANGLE;
    update_polygon_value(
            &poly_a_start, NULL, false, false, UOS_MAX_FLOAT64);

    poly_a_end.vertex_num = 4;
    poly_a_end.vertexes[0].x = 1003983.489355;
    poly_a_end.vertexes[0].y = 3407265.823461;

    poly_a_end.vertexes[1].x = 1003983.504583;
    poly_a_end.vertexes[1].y = 3407262.823499;

    poly_a_end.vertexes[2].x = 1003986.504545;
    poly_a_end.vertexes[2].y = 3407262.838728;

    poly_a_end.vertexes[3].x = 1003986.489316;
    poly_a_end.vertexes[3].y = 3407265.838689;
    poly_a_end.type = POLYGON_RECTANGLE;
    update_polygon_value(
            &poly_a_end, NULL, false, false, UOS_MAX_FLOAT64);

    poly_b_start.vertex_num = 4;
    poly_b_start.vertexes[0].x = 1003977.554392;
    poly_b_start.vertexes[0].y = 3407267.289399;

    poly_b_start.vertexes[1].x = 1003974.554462;
    poly_b_start.vertexes[1].y = 3407267.268827;

    poly_b_start.vertexes[2].x = 1003974.575034;
    poly_b_start.vertexes[2].y = 3407264.268898;

    poly_b_start.vertexes[3].x = 1003977.574963;
    poly_b_start.vertexes[3].y = 3407264.289469;

    poly_b_start.type = POLYGON_RECTANGLE;
    update_polygon_value(
            &poly_b_start, NULL, false, false, UOS_MAX_FLOAT64);

    poly_b_end.vertex_num = 4;
    poly_b_end.vertexes[0].x = 1003977.622964;
    poly_b_end.vertexes[0].y = 3407257.289633;

    poly_b_end.vertexes[1].x = 1003974.623034;
    poly_b_end.vertexes[1].y = 3407257.269063;

    poly_b_end.vertexes[2].x = 1003974.643605;
    poly_b_end.vertexes[2].y = 3407254.269133;

    poly_b_end.vertexes[3].x = 1003977.643535;
    poly_b_end.vertexes[3].y = 3407254.289704;
    poly_b_end.type = POLYGON_RECTANGLE;
    update_polygon_value(
            &poly_b_end, NULL, false, false, UOS_MAX_FLOAT64);
#endif

    bool is_collision;
    doubledist_a, dist_b;
    Position2D pos_a, pos_b;
    gjk2d_shape_cast(&is_collision, &dist_a, &dist_b, &pos_a, &pos_b,
            &poly_a_start, &poly_a_end, &poly_b_start, &poly_b_end);

#if 0
    printf("a.start->end ------ b.start->end\n");
    printf("hit spot A [%.3f, %.3f] \n", pos_a.x, pos_a.y);
    printf("hit spot B [%.3f, %.3f] \n", pos_b.x, pos_b.y);
    printf("dis a %.3f \n", dist_a);
    printf("dis b %.3f \n", dist_b);
    printf("is collision %d\n", is_collision);
#endif

    gjk2d_shape_cast(&is_collision, &dist_a, &dist_b, &pos_a, &pos_b,
            &poly_a_start, &poly_a_end, &poly_b_end, &poly_b_start);

#if 0
    printf("a.start->end ------ b.end->start\n");
    printf("hit spot A [%.3f, %.3f] \n", pos_a.x, pos_a.y);
    printf("hit spot B [%.3f, %.3f] \n", pos_b.x, pos_b.y);
    printf("dis a %.3f \n", dist_a);
    printf("dis b %.3f \n", dist_b);
    printf("is collision %d\n", is_collision);
#endif

    gjk2d_shape_cast(&is_collision, &dist_a, &dist_b, &pos_a, &pos_b,
            &poly_a_end, &poly_a_start, &poly_b_end, &poly_b_start);

#if 0
    printf("a.end->start ------ b.end->start\n");
    printf("hit spot A [%.3f, %.3f] \n", pos_a.x, pos_a.y);
    printf("hit spot B [%.3f, %.3f] \n", pos_b.x, pos_b.y);
    printf("dis a %.3f \n", dist_a);
    printf("dis b %.3f \n", dist_b);
    printf("is collision %d\n", is_collision);
#endif

    gjk2d_shape_cast(&is_collision, &dist_a, &dist_b, &pos_a, &pos_b,
            &poly_a_end, &poly_a_start, &poly_b_start, &poly_b_end);

#if 0
    printf("a.end->start ------ b.start->end\n");
    printf("hit spot A [%.3f, %.3f] \n", pos_a.x, pos_a.y);
    printf("hit spot B [%.3f, %.3f] \n", pos_b.x, pos_b.y);
    printf("dis a %.3f \n", dist_a);
    printf("dis b %.3f \n", dist_b);
    printf("is collision %d\n", is_collision);
#endif

    Pose2Dbase_pose;
    base_pose.pos.x = poly_a_start.vertexes[0].x;
    base_pose.pos.y = poly_a_start.vertexes[0].y;
    base_pose.theta = 0.0;

    viz2d_draw_polygon(
            &image_handle, &poly_a_start, &base_pose, viz2d_colors_red);

    viz2d_draw_polygon(
            &image_handle, &poly_a_end, &base_pose, viz2d_colors_black);

    viz2d_draw_polygon(
            &image_handle, &poly_b_start, &base_pose, UVIZ_COLORS_BLUE);
    viz2d_draw_polygon(
            &image_handle, &poly_b_end, &base_pose, viz2d_colors_green);

    viz2d_display(&image_handle);
    cvWaitKey(100); /* wait 2 seconds for show */

    ret = viz2d_shutdown(&image_handle);
    EXPECT_EQ(ret, 0);
}

#endif