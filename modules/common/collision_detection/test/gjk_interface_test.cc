/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include <gtest/gtest.h>
#include "./../collision_interface.h"
#include "./../gjk2d_interface.h"
#include "modules/common/math/polygon_base.h"

using namespace apollo;

TEST(gjk2d_collision_interface, test01)
{
    /* function test: interface to detect collision. */
    int ret;
    bool is_collision;
    Polygon2D polygon_veh;
    Polygon2D polygon_obj;

    init_polygon(&polygon_veh);
    init_polygon(&polygon_obj);

    /* 1.Collision: the obstacle is inside the vehicle. */
    polygon_veh.vertex_num = 4;
    polygon_veh.type = POLYGON_RECTANGLE;
    polygon_veh.vertexes[0].x = 1.0;
    polygon_veh.vertexes[0].y = 0.0;

    polygon_veh.vertexes[1].x = 1.0;
    polygon_veh.vertexes[1].y = 1.0;

    polygon_veh.vertexes[2].x = 0.0;
    polygon_veh.vertexes[2].y = 1.0;

    polygon_veh.vertexes[3].x = 0.0;
    polygon_veh.vertexes[3].y = 0.0;

    polygon_veh.center_pt.x = 0.5;
    polygon_veh.center_pt.y = 0.5;
    polygon_veh.radius = apollo_sqrt(2)/2.0;

    polygon_obj.vertex_num = 3;
    polygon_obj.vertexes[0].x = 0.0;
    polygon_obj.vertexes[0].y = 0.0;

    polygon_obj.vertexes[1].x = 1.0;
    polygon_obj.vertexes[1].y = 0.0;

    polygon_obj.vertexes[2].x = 0.0;
    polygon_obj.vertexes[2].y = 1.0;
    polygon_obj.center_pt.x = 0.0;
    polygon_obj.center_pt.y = 0.0;
    polygon_obj.radius = 1.0;

    ret = gjk2d_collision_interface(&is_collision, &polygon_veh, &polygon_obj);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(is_collision, true);

     /* 2. No Collsion. */
    polygon_obj.vertex_num = 3;
    polygon_obj.vertexes[0].x = 3.0;
    polygon_obj.vertexes[0].y = 0.0;

    polygon_obj.vertexes[1].x = 3.0;
    polygon_obj.vertexes[1].y = 1.0;

    polygon_obj.vertexes[2].x = 2.0;
    polygon_obj.vertexes[2].y = 1.0;

    polygon_obj.center_pt.x = 3.0;
    polygon_obj.center_pt.y = 0.0;
    polygon_obj.radius = apollo_sqrt(2);
    ret = gjk2d_collision_interface(&is_collision, &polygon_veh, &polygon_obj);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(is_collision, false);

    /* 2.  Collsion. */
    polygon_obj.vertex_num = 3;
    polygon_obj.vertexes[0].x = 3.0;
    polygon_obj.vertexes[0].y = 0.0;

    polygon_obj.vertexes[1].x = 0.0;
    polygon_obj.vertexes[1].y = 1.0;

    polygon_obj.vertexes[2].x = -1.0;
    polygon_obj.vertexes[2].y = -0.3;

    polygon_obj.center_pt.x = 3.0;
    polygon_obj.center_pt.y = 0.0;
    polygon_obj.radius = apollo_sqrt(2);
    ret = gjk2d_collision_interface(&is_collision, &polygon_veh, &polygon_obj);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(is_collision, 1);

    /* 2. no Collsion. */
    polygon_obj.vertex_num = 3;
    polygon_obj.vertexes[0].x = 3.0;
    polygon_obj.vertexes[0].y = 0.0;

    polygon_obj.vertexes[1].x = 0.0;
    polygon_obj.vertexes[1].y = 1.0;

    polygon_obj.vertexes[2].x = -1.0;
    polygon_obj.vertexes[2].y = -0.3;

    polygon_obj.center_pt.x = 3.0;
    polygon_obj.center_pt.y = 0.0;
    polygon_obj.radius = apollo_sqrt(2);


    polygon_veh.vertex_num = 4;
    polygon_veh.type = POLYGON_RECTANGLE;
    polygon_veh.vertexes[0].x = 3.01;
    polygon_veh.vertexes[0].y = -1.0;

    polygon_veh.vertexes[1].x = 4.0;
    polygon_veh.vertexes[1].y = -2.0;

    polygon_veh.vertexes[2].x = 4.0;
    polygon_veh.vertexes[2].y = 1.0;

    polygon_veh.vertexes[3].x = 3.5;
    polygon_veh.vertexes[3].y = 0.5;

    ret = gjk2d_collision_interface(&is_collision, &polygon_veh, &polygon_obj);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(is_collision, 0);


    /* 2.  Collsion. */
    polygon_obj.vertex_num = 3;
    polygon_obj.vertexes[0].x = 3.0;
    polygon_obj.vertexes[0].y = 0.0;

    polygon_obj.vertexes[1].x = 0.0;
    polygon_obj.vertexes[1].y = 1.0;

    polygon_obj.vertexes[2].x = -1.0;
    polygon_obj.vertexes[2].y = -0.3;

    polygon_obj.center_pt.x = 3.0;
    polygon_obj.center_pt.y = 0.0;
    polygon_obj.radius = apollo_sqrt(2);


    polygon_veh.vertex_num = 4;
    polygon_veh.type = POLYGON_RECTANGLE;
    polygon_veh.vertexes[0].x = 3.0;
    polygon_veh.vertexes[0].y = -1.0;

    polygon_veh.vertexes[1].x = 4.0;
    polygon_veh.vertexes[1].y = -2.0;

    polygon_veh.vertexes[2].x = 4.1;
    polygon_veh.vertexes[2].y = 1.0;

    polygon_veh.vertexes[3].x = 3.0;
    polygon_veh.vertexes[3].y = 0.5;

    ret = gjk2d_collision_interface(&is_collision, &polygon_veh, &polygon_obj);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(is_collision, 1);
}

TEST(gjk2d_distance_interface, test01)
{
    /* function test: interface to detect collision with distance. */
    int ret;
    bool is_collision;
    double dist;
    Polygon2D polygon_veh;
    Polygon2D polygon_obj;

    init_polygon(&polygon_veh);
    init_polygon(&polygon_obj);

    /* 1.Collision: the obstacle is inside the vehicle. */
    polygon_veh.vertex_num = 4;
    polygon_veh.type = POLYGON_RECTANGLE;
    polygon_veh.vertexes[0].x = 1.0;
    polygon_veh.vertexes[0].y = 0.0;

    polygon_veh.vertexes[1].x = 1.0;
    polygon_veh.vertexes[1].y = 1.0;

    polygon_veh.vertexes[2].x = 0.0;
    polygon_veh.vertexes[2].y = 1.0;

    polygon_veh.vertexes[3].x = 0.0;
    polygon_veh.vertexes[3].y = 0.0;

    polygon_veh.center_pt.x = 0.5;
    polygon_veh.center_pt.y = 0.5;
    polygon_veh.radius = apollo_sqrt(2)/2.0;

    polygon_obj.vertex_num = 3;
    polygon_obj.vertexes[0].x = 0.0;
    polygon_obj.vertexes[0].y = 0.0;

    polygon_obj.vertexes[1].x = 1.0;
    polygon_obj.vertexes[1].y = 0.0;

    polygon_obj.vertexes[2].x = 0.0;
    polygon_obj.vertexes[2].y = 1.0;
    polygon_obj.center_pt.x = 0.0;
    polygon_obj.center_pt.y = 0.0;
    polygon_obj.radius = 1.0;

    ret = gjk2d_distance_interface(&is_collision, &dist,
            &polygon_veh, &polygon_obj);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(is_collision, true);
    EXPECT_EQ(apollo_fequal(dist, 0.0), true);

     /* 2. No Collsion. */
    polygon_obj.vertex_num = 3;
    polygon_obj.vertexes[0].x = 3.0;
    polygon_obj.vertexes[0].y = 0.0;

    polygon_obj.vertexes[1].x = 3.0;
    polygon_obj.vertexes[1].y = 1.0;

    polygon_obj.vertexes[2].x = 2.0;
    polygon_obj.vertexes[2].y = 1.0;

    polygon_obj.center_pt.x = 2.0;
    polygon_obj.center_pt.y = 1.0;
    polygon_obj.radius = apollo_sqrt(2);
    ret = gjk2d_distance_interface(&is_collision, &dist,
            &polygon_veh, &polygon_obj);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(is_collision, false);
    EXPECT_EQ(apollo_fequal(dist, 1.0), true);
}

TEST(gjk2d_collision_detecting_with_dist_thresh, test01)
{
    int ret;
    bool is_collision;
    double dist;
    Polygon2D polygon_veh;
    Polygon2D polygon_obj;
    double dist_thresh;

    init_polygon(&polygon_veh);
    init_polygon(&polygon_obj);

    /* 1.Collision: the obstacle is inside the vehicle. */
    dist_thresh = 0.5;
    polygon_veh.vertex_num = 4;
    polygon_veh.type = POLYGON_RECTANGLE;
    polygon_veh.vertexes[0].x = 1.0;
    polygon_veh.vertexes[0].y = 0.0;

    polygon_veh.vertexes[1].x = 1.0;
    polygon_veh.vertexes[1].y = 1.0;

    polygon_veh.vertexes[2].x = 0.0;
    polygon_veh.vertexes[2].y = 1.0;

    polygon_veh.vertexes[3].x = 0.0;
    polygon_veh.vertexes[3].y = 0.0;

    polygon_veh.center_pt.x = 0.5;
    polygon_veh.center_pt.y = 0.5;
    polygon_veh.radius = apollo_sqrt(2)/2.0;

    polygon_obj.vertex_num = 3;
    polygon_obj.vertexes[0].x = 0.0;
    polygon_obj.vertexes[0].y = 0.0;

    polygon_obj.vertexes[1].x = 1.0;
    polygon_obj.vertexes[1].y = 0.0;

    polygon_obj.vertexes[2].x = 0.0;
    polygon_obj.vertexes[2].y = 1.0;
    polygon_obj.center_pt.x = 0.0;
    polygon_obj.center_pt.y = 0.0;
    polygon_obj.radius = 1.0;
    ret = gjk2d_collision_detecting_with_dist_thresh(&is_collision, &dist,
            &polygon_veh, &polygon_obj,dist_thresh);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(is_collision, true);
    EXPECT_EQ(apollo_fequal(dist, 0.0), true);

    /* 2. No Collsion. */
    polygon_obj.vertex_num = 3;
    polygon_obj.vertexes[0].x = 3.0;
    polygon_obj.vertexes[0].y = 0.0;

    polygon_obj.vertexes[1].x = 3.0;
    polygon_obj.vertexes[1].y = 1.0;

    polygon_obj.vertexes[2].x = 2.0;
    polygon_obj.vertexes[2].y = 1.0;

    polygon_obj.center_pt.x = 2.0;
    polygon_obj.center_pt.y = 1.0;
    polygon_obj.radius = apollo_sqrt(2);
    ret = gjk2d_collision_detecting_with_dist_thresh(&is_collision, &dist,
            &polygon_veh, &polygon_obj,dist_thresh);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(is_collision, false);
    EXPECT_EQ(apollo_fequal(dist, 1.0), true);
}

TEST(sat_gjk2d_test, collision_dist_paris_test01)
{
    uint32_t i;
    bool is_collision;
    double dist = 1000000;
    int ret;

    Position2D pos1, pos2;
    Polygon2D polygon1, polygon2;

    Position2D P_pos[4] = {
        {-77175.537, 4389771.074},
        {-77176.170, 4389769.176},
        {-77173.846, 4389768.401},
        {-77173.213, 4389770.299}
    };

    Position2D Q_pos[7] = {
        {-77172.639, 4389770.635},
        {-77172.922, 4389770.336},
        {-77172.987, 4389769.822},

        {-77172.966, 4389769.753},
        {-77172.066, 4389769.512},
        {-77172.265, 4389770.556},
        {-77172.333, 4389770.724},
    };

    polygon1.vertex_num = 4;
    polygon1.center_pt.x = 0.0;
    polygon1.center_pt.y = 0.0;
    for (i = 0; i < polygon1.vertex_num; i++)
    {
        polygon1.vertexes[i].x = P_pos[i].x;
        polygon1.vertexes[i].y = P_pos[i].y;

        polygon1.center_pt.x += polygon1.vertexes[i].x;
        polygon1.center_pt.y += polygon1.vertexes[i].y;
    }
    polygon1.center_pt.x /= polygon1.vertex_num;
    polygon1.center_pt.y /= polygon1.vertex_num;

    polygon2.vertex_num = 7;
    polygon2.center_pt.x = 0.0;
    polygon2.center_pt.y = 0.0;
    for (i = 0; i < polygon2.vertex_num; i++)
    {
        polygon2.vertexes[i].x = Q_pos[i].x;
        polygon2.vertexes[i].y = Q_pos[i].y;
        polygon2.center_pt.x += polygon2.vertexes[i].x;
        polygon2.center_pt.y += polygon2.vertexes[i].y;
    }
    polygon2.center_pt.x /= polygon2.vertex_num;
    polygon2.center_pt.y /= polygon2.vertex_num;

    ret = gjk2d_collision_detecting_with_dist_pair(&is_collision,
           &dist, &pos1, &pos2,
           &polygon1, &polygon2);
    EXPECT_EQ(0, ret);
    EXPECT_EQ(false, is_collision);
    EXPECT_NEAR(0.284059, dist, 1e-5);
    EXPECT_NEAR(-77173.21300, pos1.x, 1e-5);
    EXPECT_NEAR(4389770.29900, pos1.y, 1e-5);
    EXPECT_NEAR(-77172.93119, pos2.x, 1e-5);
    EXPECT_NEAR(4389770.26336, pos2.y, 1e-5);
}