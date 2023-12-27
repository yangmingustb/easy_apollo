/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include <gtest/gtest.h>
#include "modules/common/math/polygon_base.h"

using namespace apollo;

/* Determine whether two points are the same */
static bool
is_point_3d_same(Position2D &point_a, Position2D &point_b)
{
    int32_t i;
    double*pt_a, *pt_b;
    pt_a = &point_a.x;
    pt_b = &point_b.x;
    for (i = 0; i < 3; i++)
    {
        if (!apollo_fequal(*pt_a, *pt_b))
        {
            printf("The %dth point in the two 3D points is not the same\n", i+1);
            return false;
        }
        pt_a++;
        pt_b++;
    }
    return true;
}

/* Determine whether two polygons are the same */
static bool
is_polygon_same(Polygon2D *p1, Polygon2D *p2)
{
    uint32_t i;
    bool same = true;

    if (p1->vertex_num != p2->vertex_num)
    {
        printf("polygon vertex num diff %d %d \n",
                p1->vertex_num, p2->vertex_num);
    }
    for (i = 0; i < p1->vertex_num; i++)
    {
        same = same && (apollo_fequal(p1->vertexes[i].x, p2->vertexes[i].x) &&
                    apollo_fequal(p1->vertexes[i].y, p2->vertexes[i].y));
        if (!same)
        {
            printf("polygon vertex [%d]  pos diff (%.2f %.2f),"
                    "(%.2f %.2f),\n",
                    i, p1->vertexes[i].x, p1->vertexes[i].x,
                    p2->vertexes[i].x, p2->vertexes[i].y);
        }
    }

    return same;
}

TEST(init_polygon, test01)
{
    /* Wrong input,input is a null pointer. */
    int ret;
    ret = init_polygon(NULL);
    EXPECT_EQ(ret, -1);
}

TEST(init_polygon, test02)
{
    /* function test: initialize the polygon */
    int32_t i;
    Polygon2D polygon;
    int ret;

    polygon.radius = 1.0;
    polygon.vertex_num = 2;
    polygon.vertexes[0].x = 2.0;
    polygon.type = POLYGON_RECTANGLE;

    /* Test each parameter of initialization in the struct of polygon */
    ret = init_polygon(&polygon);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(polygon.type, POLYGON_ARBITRARY);
    EXPECT_EQ(polygon.vertex_num, 0);
    EXPECT_DOUBLE_EQ(polygon.radius, 0.0);

    Position2D point = {0.0, 0.0};
    EXPECT_EQ(is_point_3d_same(polygon.center_pt, point), true);

    for (i = 0; i < MAX_POLYGON_VERTEX_NUM; i++)
    {
        EXPECT_EQ(is_point_3d_same(polygon.vertexes[i], point), true);
        EXPECT_EQ(is_point_3d_same(polygon.axises[i], point), true);
    }
    // init_polygon() and memset() have the same function
    // Polygon2D polygon_B;
    // memset(&polygon_B, 0, sizeof(polygon_t));
    // EXPECT_TRUE(is_polygon_same(&polygon, &polygon_B));
}

TEST(polygon_assign, test01)
{
    /* Wrong input,input is a null pointer. */
    int ret;
    Polygon2D des_poly;
    Polygon2D src_poly;

    ret = polygon_assign(&des_poly, NULL);
    EXPECT_EQ(ret, -1);

    ret = polygon_assign(NULL, &src_poly);
    EXPECT_EQ(ret, -1);
}

TEST(polygon_assign, test02)
{
    /* function test */
    int ret;
    Polygon2D des_poly;
    Polygon2D src_poly;

    /* use the function of init_polygon() to ensure all parameters are
     * initialized in the struct of des_poly and src_poly.
     * */
    ret = init_polygon(&des_poly);
    EXPECT_EQ(ret, 1);
    ret = init_polygon(&src_poly);
    EXPECT_EQ(ret, 1);

    src_poly.type = POLYGON_RECTANGLE;
    src_poly.vertex_num = 6;
    src_poly.vertexes[0].x = 1.0;
    src_poly.vertexes[0].y = 2.0;
    src_poly.radius = 3.0;

    ret = polygon_assign(&des_poly, &src_poly);
    EXPECT_TRUE(is_polygon_same(&des_poly, &src_poly));
}

TEST(update_polygon_value, test01)
{
    /* Wrong input,input is a null pointer. */
    int ret;
    Polygon2D polygon;
    Pose2D center_pose;
    bool use_center_pose;
    bool radius_known;
    double radius;

    radius = 0.0;
    use_center_pose = false;
    radius_known = false;

    ret = update_polygon_value(NULL, &center_pose,
            use_center_pose, radius_known, radius);
    EXPECT_EQ(ret, -1);

    /* Illegal input data */
    ret = init_polygon(&polygon);
    EXPECT_EQ(ret, 1);

    polygon.vertex_num = 13;
    ret = update_polygon_value(&polygon, &center_pose,
            use_center_pose, radius_known, radius);
    EXPECT_FALSE(is_polygon_legal(&polygon));
    EXPECT_EQ(ret, -1);

    polygon.vertex_num = 0;
    ret = update_polygon_value(&polygon, &center_pose,
            use_center_pose, radius_known, radius);
    EXPECT_FALSE(is_polygon_legal(&polygon));
    EXPECT_EQ(ret, -1);
}

TEST(update_polygon_value, test02)
{
    int ret;
    Polygon2D polygon;
    bool use_center_pose;
    bool radius_known;
    double radius;

    radius = 0.0;
    ret = init_polygon(&polygon);
    EXPECT_EQ(ret, 0);

    /* Input rectangle coordinates */
    polygon.type = POLYGON_RECTANGLE;
    polygon.vertex_num = 4;
    polygon.vertexes[0].x = 3.0;
    polygon.vertexes[1].x = 3.0;
    polygon.vertexes[1].y = 2.0;
    polygon.vertexes[2].y = 2.0;

    /* Radius is unknown */
    use_center_pose = false;
    radius_known = false;
    ret = update_polygon_value(&polygon, NULL, use_center_pose,
            radius_known, radius);
    EXPECT_EQ(ret, 1);

    Position2D center_point = {1.5, 1.0};
    EXPECT_TRUE(is_point_3d_same(polygon.center_pt, center_point));

    EXPECT_DOUBLE_EQ(polygon.radius, apollo_sqrt(3.25));
    EXPECT_DOUBLE_EQ(polygon.axises[0].x, -2.0);
    EXPECT_DOUBLE_EQ(polygon.axises[0].y, 0.0);
    EXPECT_DOUBLE_EQ(polygon.axises[1].x, 0.0);
    EXPECT_DOUBLE_EQ(polygon.axises[1].y, -3.0);

    /* Radius is known */
    radius_known = true;
    ret = update_polygon_value(&polygon, NULL, use_center_pose,
            radius_known, 3);
    EXPECT_EQ(ret, 1);
    EXPECT_DOUBLE_EQ(polygon.radius, 3.0);
}

TEST(update_polygon_value, test03)
{
    int ret;
    Polygon2D polygon;
    Pose2D center_pose;

    center_pose = {{1.0, 1.0}, {0.0}};
    ret = init_polygon(&polygon);
    EXPECT_EQ(ret, 1);
    EXPECT_EQ(polygon.type, POLYGON_ARBITRARY);

    /* Input polyhedron coordinates. */
    polygon.vertex_num = 5;
    polygon.vertexes[0].x = 3.0;
    polygon.vertexes[1].x = 3.0;
    polygon.vertexes[1].y = 2.0;
    polygon.vertexes[2].x = 1.5;
    polygon.vertexes[2].y = 3.0;
    polygon.vertexes[3].y = 2.0;

    /* There are center point and radius with information. */
    ret = update_polygon_value(&polygon, &center_pose, true,
            true, 3);
    EXPECT_EQ(ret, 1);

    Position2D center_point = {1.0, 1.0};
    EXPECT_EQ(is_point_3d_same(polygon.center_pt, center_point), true);

    EXPECT_DOUBLE_EQ(polygon.axises[0].x, -2.0);
    EXPECT_DOUBLE_EQ(polygon.axises[0].y, 0.0);
    EXPECT_DOUBLE_EQ(polygon.axises[1].x, -1.0);
    EXPECT_DOUBLE_EQ(polygon.axises[1].y, -1.5);
    EXPECT_DOUBLE_EQ(polygon.radius, 3);

    /* There is a center point with information without radius. */
    ret = update_polygon_value(&polygon, &center_pose, true,
            false, 3);
    EXPECT_EQ(ret, 1);
    EXPECT_EQ(is_point_3d_same(polygon.center_pt, center_point), true);

    EXPECT_DOUBLE_EQ(polygon.radius, apollo_sqrt(5));
    EXPECT_DOUBLE_EQ(polygon.axises[0].x, -2.0);
    EXPECT_DOUBLE_EQ(polygon.axises[0].y, 0.0);
    EXPECT_DOUBLE_EQ(polygon.axises[1].x, -1.0);
    EXPECT_DOUBLE_EQ(polygon.axises[1].y, -1.5);

    /* There are radius and center point without information. */
    ret = update_polygon_value(&polygon, &center_pose, false,
            true, 3);
    EXPECT_EQ(ret, 1);

    center_point = {3.0, 0.0};
    EXPECT_EQ(is_point_3d_same(polygon.center_pt, center_point), true);

    EXPECT_DOUBLE_EQ(polygon.radius, 3);

    /* There is a center point without information and radius. */
    ret = update_polygon_value(&polygon, NULL, true,
            false, 3);
    EXPECT_EQ(ret, 1);
    EXPECT_EQ(is_point_3d_same(polygon.center_pt, center_point), true);

    EXPECT_DOUBLE_EQ(polygon.radius, apollo_sqrt(13));
}

TEST(update_polygon_value, test04)
{
    /* corner cases: Point overlap and Abnormal polygon. */
    int ret;
    Polygon2D polygon;
    Pose2D center_pose;
    uint32_t i;

    /* Declare ret as the opposite of the expected result. */
    center_pose = {{2.0, 1.0}, {0.0}};
    ret = init_polygon(&polygon);
    EXPECT_EQ(ret, 1);
    EXPECT_EQ(polygon.type, POLYGON_ARBITRARY);

    /* Input polyhedron information: 4 points overlap. */
    polygon.vertex_num = 4;
    polygon.type = POLYGON_RECTANGLE;
    for (i = 0; i < polygon.vertex_num; i++)
    {
        polygon.vertexes[i].x = 1.0;
        polygon.vertexes[i].y = 1.0;
    }
    ret = update_polygon_value(&polygon, &center_pose, true,
            false, 3);
    EXPECT_EQ(ret, 1);

    Position2D center_point = {1.0, 1.0};
    EXPECT_EQ(is_point_3d_same(polygon.center_pt, center_point), true);

    EXPECT_DOUBLE_EQ(polygon.radius, 0);

    /* Input polyhedron information: 3 points overlap. */
    polygon.vertex_num = 4;
    polygon.type = POLYGON_RECTANGLE;
    for (i = 0; i < polygon.vertex_num - 1; i++)
    {
        polygon.vertexes[i].x = 1.0;
        polygon.vertexes[i].y = 1.0;
    }
    polygon.vertexes[polygon.vertex_num - 1].x = 0.0;
    polygon.vertexes[polygon.vertex_num - 1].y = 0.0;

    ret = update_polygon_value(&polygon, &center_pose, true,
            false, 3);
    EXPECT_EQ(ret, 1);
    EXPECT_EQ(is_point_3d_same(polygon.center_pt, center_point), true);

    EXPECT_DOUBLE_EQ(polygon.radius, 0.0);

    /* Input polyhedron information: 2 points overlap. */
    ret = init_polygon(&polygon);
    polygon.type = POLYGON_RECTANGLE;
    polygon.vertex_num = 4;
    polygon.vertexes[0].x = 3.0;
    polygon.vertexes[1].x = 3.0;
    polygon.vertexes[1].y = 2.0;
    polygon.vertexes[2].x = 0.0;
    polygon.vertexes[2].y = 0.0;
    polygon.vertexes[3].x = 0.0;
    polygon.vertexes[3].y = 0.0;
    ret = update_polygon_value(&polygon, &center_pose, true,
            false, 3);
    EXPECT_EQ(ret, 1);

    center_point = {1.5, 0.0};
    EXPECT_EQ(is_point_3d_same(polygon.center_pt, center_point), true);

    EXPECT_DOUBLE_EQ(polygon.radius, 1.5);
}

TEST(generate_rect_polygon, test01)
{
    /* Wrong input,input is a null pointer. */
    int ret;
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;

    ret = generate_rect_polygon(NULL, min_x, min_y, max_x, max_y);
    EXPECT_EQ(ret, 0);
}

TEST(generate_rect_polygon, test02)
{
    int ret;
    double min_x = 4.0;
    double min_y = 4.0;
    double max_x = 6.0;
    double max_y = 6.0;
    Polygon2D polygon;

    ret = init_polygon(&polygon);
    EXPECT_EQ(ret, 1);

    ret = generate_rect_polygon(&polygon, min_x, min_y, max_x, max_y);
    EXPECT_EQ(ret, 1);

    Position2D vertexes1 = {6.0, 6.0};
    EXPECT_EQ(is_point_3d_same(polygon.vertexes[0], vertexes1), true);

    Position2D vertexes2 = {4.0, 6.0};
    EXPECT_EQ(is_point_3d_same(polygon.vertexes[1], vertexes2), true);

    Position2D vertexes3 = {4.0, 4.0};
    EXPECT_EQ(is_point_3d_same(polygon.vertexes[2], vertexes3), true);

    Position2D vertexes4 = {6.0, 4.0};
    EXPECT_EQ(is_point_3d_same(polygon.vertexes[3], vertexes4), true);
}