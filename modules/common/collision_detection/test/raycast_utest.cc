/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include <gtest/gtest.h>
#include "cdl.h"
#include "collision_interface.h"
#include "gjk2d_interface.h"

using namespace apollo;

using namespace cdl;

TEST(GJKBASED_RAYCAST, HIT2)
{
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, 0.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 5.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(-10, 1);
    input.direction = cdl::Vector2r(1, 0);
    input.max_lambda = 30;
    for (size_t i = 0; i < vertices.size(); i++)
    {
        input.shape.vertices[i]=vertices[i];
    }
    input.shape.size = vertices.size();

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.GJKBasedRayCast(&input, &result);
    EXPECT_NEAR(10.000, result.hit_distance, 1e-3);
    EXPECT_NEAR(0.000, result.hit_spot(0), 1e-3);
    EXPECT_NEAR(1.000, result.hit_spot(1), 1e-3);
    EXPECT_NEAR(-1.000, result.hit_normal(0), 1e-3);
    EXPECT_NEAR(0.000, result.hit_normal(1), 1e-3);

    EXPECT_EQ(cdl::np2d::RayCastStatus::HIT, result.status);
    /** travesal version */
    gjk_solver.traversalRayCast(&input, &result);
    EXPECT_EQ(result.hit_edge_id, 3);
}

TEST(GJKBASED_RAYCAST, no_hit)
{
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, 0.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 5.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(-10, -1);
    input.direction = cdl::Vector2r(1, 0);
    input.max_lambda = 30;
    for (size_t i = 0; i < vertices.size(); i++)
    {
        input.shape.vertices[i]=vertices[i];
    }
    input.shape.size = vertices.size();

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.GJKBasedRayCast(&input, &result);

    EXPECT_EQ(cdl::np2d::RayCastStatus::NO_HIT, result.status);
    /** travesal version */
    gjk_solver.traversalRayCast(&input, &result);
    EXPECT_EQ(result.hit_edge_id, -1);

}

TEST(GJKBASED_RAYCAST, source_is_inside)
{
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, 0.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 5.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(1, 1);
    input.direction = cdl::Vector2r(1, 0);
    input.max_lambda = 30;
    for (size_t i = 0; i < vertices.size(); i++)
    {
        input.shape.vertices[i]=vertices[i];
    }
    input.shape.size = vertices.size();

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    /** travesal version */
    gjk_solver.traversalRayCast(&input, &result);

    EXPECT_EQ(cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE, result.status);
    EXPECT_EQ(result.hit_edge_id, 1);
    EXPECT_EQ(result.hit_spot(0), 5.00);
    EXPECT_EQ(result.hit_spot(1), 1.00);

}

#ifdef ENABLE_UVIZ
TEST(raycast, raycast_20211026_bug)
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

    ret = uviz_set_default_font(&font);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(font.font_type, viz2d_font_hershey_simplex);

    strncpy(win_name, "Debug raycast", MAX_WINDOW_NAME_LEN);
    win_columns = 1200;
    win_rows = 800;
    origin_columns_index = 200;
    origin_rows_index = 200;
    background_color = viz2d_colors_white;

    resolution = 0.1;
    uviz_init_image_handle(&image_handle, &font, win_name, win_columns,
            win_rows, origin_columns_index, origin_rows_index, resolution,
            background_color);
    ret = uviz_start(&image_handle);
    EXPECT_EQ(ret, 0);
    EXPECT_TRUE(NULL != image_handle.image);
    EXPECT_EQ(image_handle.image->nChannels, 3);
    EXPECT_EQ(image_handle.image->depth, 8);
    EXPECT_EQ(image_handle.image->width, win_columns);
    EXPECT_EQ(image_handle.image->height, win_rows);

    ret = uviz_draw_xy_axis(&image_handle);
    EXPECT_EQ(ret, 0);

    Polygon2D poly;

    poly.vertex_num = 4;
    poly.vertexes[0].x = -79501.396;
    poly.vertexes[0].y = 4389807.977;

    poly.vertexes[1].x = -79501.583;
    poly.vertexes[1].y = 4389806.489;

    poly.vertexes[2].x = -79499.470;
    poly.vertexes[2].y = 4389806.222;

    poly.vertexes[3].x = -79499.283;
    poly.vertexes[3].y = 4389807.711;
    poly.type = POLYGON_RECTANGLE;
    update_polygon_value(&poly, NULL, false, false, UOS_MAX_FLOAT64);

    Position2D src;
    Position2D dir;
    float32_t  ratio;
    bool is_collision;
    Position2D collision_point;
    int32_t hit_edge_id;
    src.x = -103176.897;
    src.y = 5697235.157;

    dir.x = -1.402;
    dir.y = 0.190;

    raycast_extract_collisoin_point(&src, &dir, 20, &poly, &is_collision,
            &collision_point, &hit_edge_id, &ratio, NULL);

    Pose2D base_pose;
    base_pose.pos.x = poly.vertexes[0].x;
    base_pose.pos.y = poly.vertexes[0].y;
    base_pose.theta = 0.0;

    uviz_draw_polygon(&image_handle, &poly, &base_pose,
            viz2d_colors_red);

    CvPoint pt1, pt2;
    CvScalar line_color;
    ret = uviz_get_color(&line_color, viz2d_colors_black);

    Position2D start_local;

    cvt_pos_global_to_local(&start_local, &src, &base_pose);
    Position2D end_local, end_global;

    end_global.x = src.x+dir.x;
    end_global.y = src.x+dir.y;

    cvt_pos_global_to_local(&end_local, &end_global, &base_pose);

    uviz_get_index(&image_handle, &pt1, start_local.x, start_local.y);
    uviz_get_index(&image_handle, &pt2, end_local.x, end_local.y);
    cvLine(image_handle.image, pt1, pt2, line_color, 1, CV_AA, 0);

    uviz_display(&image_handle);
    cvWaitKey(10); /* wait 2 seconds for show */

    ret = uviz_shutdown(&image_handle);
    EXPECT_EQ(ret, 0);
}

#endif

#include <gtest/gtest.h>
#include "cdl.h"
using namespace apollo;
using namespace cdl;
void printStatus(const cdl::np2d::RayCastResult &result)
{
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            printf("source is inside \n");
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            printf("no hit \n");
            break;
        case cdl::np2d::RayCastStatus::HIT:
            printf("hit \n");
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            printf("will hit \n");
            break;
        default:
            break;
    }
}

TEST(GJKBASED_TRAVERSAL_RAYCAST, HIT)
{
    // set polygon
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, 0.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 5.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(-10, 1);
    input.direction = cdl::Vector2r(1, 0);
    input.max_lambda = 30;
    for (size_t i = 0; i < vertices.size(); i++)
    {
        input.shape.vertices[i]=vertices[i];
    }
    input.shape.size = vertices.size();

    // set output
    cdl::np2d::RayCastResult result;
    cdl::np2d::GJK2D gjk_solver;

    gjk_solver.GJKBasedRayCast(&input, &result);
    EXPECT_DOUBLE_EQ(10.0, result.hit_distance);
    EXPECT_DOUBLE_EQ(0.0, result.hit_spot(0));
    EXPECT_DOUBLE_EQ(1.0, result.hit_spot(1));
    EXPECT_DOUBLE_EQ(-1.0, result.hit_normal(0));
    EXPECT_DOUBLE_EQ(0.0, result.hit_normal(1));

    EXPECT_EQ(cdl::np2d::RayCastStatus::HIT, result.status);
    /** travesal version */
    gjk_solver.traversalRayCast(&input, &result);
    EXPECT_EQ(result.hit_edge_id, 3);
}

TEST(GJKBASED_TRAVERSAL_RAYCAST, WILL_HIT)
{
    // set polygon
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, 0.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 5.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(-21.0, 1);
    input.direction = cdl::Vector2r(1, 0);
    input.max_lambda = 20.0;
    for (size_t i = 0; i < vertices.size(); i++)
    {
        input.shape.vertices[i]=vertices[i];
    }
    input.shape.size = vertices.size();

    // set output
    cdl::np2d::RayCastResult result;
    cdl::np2d::GJK2D gjk_solver;

    gjk_solver.GJKBasedRayCast(&input, &result);
    EXPECT_FLOAT_EQ(0.0, result.hit_distance);
    EXPECT_FLOAT_EQ(0.0, result.hit_spot(0));
    EXPECT_FLOAT_EQ(0.0, result.hit_spot(1));
    EXPECT_FLOAT_EQ(0.0, result.hit_normal(0));
    EXPECT_FLOAT_EQ(0.0, result.hit_normal(1));

    EXPECT_EQ(cdl::np2d::RayCastStatus::WILL_HIT, result.status);

    /** travesal version */
    gjk_solver.traversalRayCast(&input, &result);
    EXPECT_EQ(cdl::np2d::RayCastStatus::WILL_HIT, result.status);
}

TEST(GJKBASED_TRAVERSAL_RAYCAST, NO_HIT)
{
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, 0.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 5.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(-10, -1);
    input.direction = cdl::Vector2r(1, 0);
    input.max_lambda = 30;
    for (size_t i = 0; i < vertices.size(); i++)
    {
        input.shape.vertices[i]=vertices[i];
    }
    input.shape.size = vertices.size();

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.GJKBasedRayCast(&input, &result);
    EXPECT_EQ(result.hit_edge_id, -1);
    EXPECT_EQ(cdl::np2d::RayCastStatus::NO_HIT, result.status);

    /** travesal version */
    gjk_solver.traversalRayCast(&input, &result);
    EXPECT_EQ(result.hit_edge_id, -1);
    EXPECT_EQ(cdl::np2d::RayCastStatus::NO_HIT, result.status);
}

TEST(GJKBASED_TRAVERSAL_RAYCAST, SOURCE_IS_INSIDE)
{
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, 0.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 5.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(1, 1);
    input.direction = cdl::Vector2r(1, 0);
    input.max_lambda = 30;
    for (size_t i = 0; i < vertices.size(); i++)
    {
        input.shape.vertices[i]=vertices[i];
    }
    input.shape.size = vertices.size();

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;

    gjk_solver.GJKBasedRayCast(&input, &result);
    EXPECT_EQ(cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE, result.status);

    /** travesal version */
    gjk_solver.traversalRayCast(&input, &result);

    EXPECT_EQ(cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE, result.status);
    EXPECT_EQ(result.hit_edge_id, 1);
    EXPECT_EQ(result.hit_spot(0), 5.00);
    EXPECT_EQ(result.hit_spot(1), 1.00);
}

TEST(SHAPECAST, SOURCE_IS_INSIDE)
{
    cdl::np2d::ShapeCastRequest input;
    cdl::np2d::ShapeCastResult output;
    cdl::np2d::GJK2D gjk_solver;
    uint32_t size;
    real error = 0.002;

    Position2D P_pos[4] = {
        {-76996.317957,4389958.369791},
        {-76994.318135,4389958.343173},
        {-76994.285528,4389960.792956},
        {-76996.285351,4389960.819574}};

    Position2D Q_pos[4] = {
        {-76993.861329,4389955.599311},
        {-76993.610663,4389957.583541},
        {-76997.579122,4389958.084874},
        {-76997.829788,4389956.100644}};

    size = input.proxyA.size = 4;
    for (uint32_t i = 0; i < size; i++)
    {
        input.proxyA.vertices[i](0) = P_pos[i].x;
        input.proxyA.vertices[i](1) = P_pos[i].y;
    }
    input.translationA = {-0.007989, -0.599947};

    size = input.proxyB.size = 4;
    for (uint32_t i = 0; i < size; i++)
    {
        input.proxyB.vertices[i](0) = Q_pos[i].x;
        input.proxyB.vertices[i](1) = Q_pos[i].y;
    }
    input.translationB = {0.0, 0.0};

    gjk_solver.shapeCast(&output, &input);
    EXPECT_EQ(output.collided, 1);
    EXPECT_NEAR(output.distanceA, 0.443533, error);
    EXPECT_NEAR(output.collision_pointA.row(0)(0), -76996.317957, error);
    EXPECT_NEAR(output.collision_pointA.row(1)(0), 4389958.369791, error);
    EXPECT_NEAR(output.collision_pointB.row(0)(0), -76996.323863, error);
    EXPECT_NEAR(output.collision_pointB.row(1)(0), 4389957.926298, error);
}