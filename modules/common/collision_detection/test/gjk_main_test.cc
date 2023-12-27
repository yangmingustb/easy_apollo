#include <gtest/gtest.h>
#include "./../gjk2d_interface.h"
#include "./../gjk2d.h"
#include "modules/common/math/pose.h"

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

using cdl::Vector2r;
using namespace cdl::np2d;
using namespace apollo;

TEST(GJK2DTest, gjk2D_s1d_mid) {
  cdl::np2d::GJK2D solver;
  cdl::PointType p;
  // middle case
  solver.simplex_.vertices.row(0) = Vector2r(1.0, 0);
  solver.simplex_.vertices.row(1) = Vector2r(0.0, 1.0);
  solver.simplex_.num = 2;
  solver.S1D2d(solver.simplex_, p);
  EXPECT_EQ(2, solver.simplex_.num);
  EXPECT_FLOAT_EQ(0.5, solver.simplex_.lambdas(0));
  EXPECT_FLOAT_EQ(0.5, solver.simplex_.lambdas(1));
  EXPECT_FLOAT_EQ(0.5, p(0));
  EXPECT_FLOAT_EQ(0.5, p(1));
}

TEST(GJK2DTest, gjk2D_s1d_a) {
  cdl::np2d::GJK2D solver;
  cdl::PointType p;

  // middle case
  solver.simplex_.vertices.row(0) = Vector2r(4, 1.0);
  solver.simplex_.vertices.row(1) = Vector2r(10, 0);
  solver.simplex_.num = 2;
  solver.S1D2d(solver.simplex_, p);
  EXPECT_EQ(1, solver.simplex_.num);
  EXPECT_FLOAT_EQ(1.0, solver.simplex_.lambdas(0));
  EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(0)(0), p(0));
  EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(0)(1), p(1));
}

TEST(GJK2DTest, gjk2D_bs1d_a) {
  cdl::np2d::GJK2D solver;
  cdl::PointType p;

  // middle case
  solver.simplex_.vertices.row(0) = Vector2r(4, 1.0);
  solver.simplex_.vertices.row(1) = Vector2r(10, 0);
  solver.simplex_.num = 2;
  solver.S1D2d(solver.simplex_, p);
  EXPECT_EQ(1, solver.simplex_.num);
  EXPECT_FLOAT_EQ(1.0, solver.simplex_.lambdas(0));
  EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(0)(0), p(0));
  EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(0)(1), p(1));
}


TEST(GJK2DTest, gjk2D_s1d_b) {
  cdl::np2d::GJK2D solver;
  cdl::PointType p;

  // middle case
  solver.simplex_.vertices.row(0) = Vector2r(-1.0, -1.0);
  solver.simplex_.vertices.row(1) = Vector2r(-2.0, -5.0);
  solver.simplex_.num = 2;
  solver.S1D2d(solver.simplex_, p);
  EXPECT_EQ(1, solver.simplex_.num);
  EXPECT_FLOAT_EQ(1.0, solver.simplex_.lambdas(0));
  EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(0)(0), p(0));
  EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(0)(1), p(1));
}


TEST(GJK2DTest, gjk2D_s1d_overlap_a) {
  cdl::np2d::GJK2D solver;
  cdl::PointType p;

  // middle case
  solver.simplex_.vertices.row(0) = Vector2r(0, 0);
  solver.simplex_.vertices.row(1) = Vector2r(-2.0, -5.0);
  solver.simplex_.num = 2;
  solver.S1D2d(solver.simplex_, p);
  EXPECT_EQ(1, solver.simplex_.num);
  EXPECT_FLOAT_EQ(1.0, solver.simplex_.lambdas(0));
  EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(0)(0), p(0));
  EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(0)(1), p(1));
}

TEST(GJK2DTest, gjk2D_s1d_overlap_b) {
  cdl::np2d::GJK2D solver;
  cdl::PointType p;

  // middle case
  solver.simplex_.vertices.row(0) = Vector2r(-1.0, -1.0);
  solver.simplex_.vertices.row(1) = Vector2r(0, 0);
  solver.simplex_.num = 2;
  solver.S1D2d(solver.simplex_, p);
  EXPECT_EQ(1, solver.simplex_.num);
  EXPECT_FLOAT_EQ(1, solver.simplex_.lambdas(0));
  EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(1)(0), p(0));
}

TEST(GJK2DTest, gjk2D_s1d_degenerated) {
  cdl::np2d::GJK2D solver;
  cdl::PointType p;

  // middle case
  solver.simplex_.vertices.row(0) = Vector2r(-1.0, -1.0);
  solver.simplex_.vertices.row(1) = Vector2r(-1.0, -1.0);
  solver.simplex_.num = 2;
  solver.S1D2d(solver.simplex_, p);
  EXPECT_EQ(1, solver.simplex_.num);
  EXPECT_FLOAT_EQ(1.0, solver.simplex_.lambdas(0));
  EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(0)(0), p(0));
  EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(0)(1), p(1));
}


// TEST(GJK2DTest, gjk2D_s2d_a) {
//   cdl::np2d::GJK2D solver;
//   cdl::PointType p;

//   // middle case
//   solver.simplex_.vertices.row(0) = Vector2r(1.0, 1.0);
//   solver.simplex_.vertices.row(1) = Vector2r(10.0, 0.0);
//   solver.simplex_.vertices.row(2) = Vector2r(0, 10.0);
//   solver.simplex_.num = 3;

//   solver.S2D2d(solver.simplex_, p);

//   EXPECT_EQ(1, solver.simplex_.num);
//   EXPECT_FLOAT_EQ(1.0, solver.simplex_.lambdas(0));
//   EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(0)(0), p(0));
//   EXPECT_FLOAT_EQ(solver.simplex_.vertices.row(0)(1), p(1));
// }

TEST(GJK2DTest, gjk2D_s2d_edge) {
  cdl::np2d::GJK2D solver;
  cdl::PointType p;

  // middle case
  solver.simplex_.vertices.row(0) = Vector2r(1.0, 0.0);
  solver.simplex_.vertices.row(1) = Vector2r(10.0, 0.0);
  solver.simplex_.vertices.row(2) = Vector2r(0, 10.0);
  solver.simplex_.num = 3;
  solver.S2D2d(solver.simplex_, p);
  EXPECT_EQ(2, solver.simplex_.num);
  EXPECT_NEAR(0.990099, solver.simplex_.lambdas(0), 1e-5);
  EXPECT_NEAR(0.00990093, solver.simplex_.lambdas(1), 1e-5);
}

// TEST(GJK2DTest, gjk2D_s2d_edge_and_righted_angle) {
//   cdl::np2d::GJK2D solver;
//   cdl::PointType p;

//   // middle case
//   solver.simplex_.vertices.row(0) = Vector2r(0.0, 10.0);
//   solver.simplex_.vertices.row(1) = Vector2r(-20.0, -10.0);
//   solver.simplex_.vertices.row(2) = Vector2r(0.0, -10.0);
//   solver.simplex_.num = 3;

//   solver.S2D2d(solver.simplex_, p);

//   std::cout<<"lambda:"<<solver.simplex_.lambdas<<std::endl;
//   std::cout<<"vertice:"<<solver.simplex_.vertices<<std::endl;

//   EXPECT_EQ(2, solver.simplex_.num);
//   // EXPECT_NEAR(0.990099, solver.simplex_.lambdas(0), 1e-5);
//   // EXPECT_NEAR(0.00990093, solver.simplex_.lambdas(1), 1e-5);
// }

TEST(GJK2DTest, gjk2D_s2d_origin_vertice) {
  cdl::np2d::GJK2D solver;
  cdl::PointType p;

  // middle case
  solver.simplex_.vertices.row(0) = Vector2r(2.0, -2.0);
  solver.simplex_.vertices.row(1) = Vector2r(-20.0, 0.0);
  solver.simplex_.vertices.row(2) = Vector2r(-2, 2.0);
  solver.simplex_.num = 3;
  solver.S2D2d(solver.simplex_, p);
  EXPECT_EQ(2, solver.simplex_.num);
  // EXPECT_NEAR(0.990099, solver.simplex_.lambdas(0), 1e-5);
  // EXPECT_NEAR(0.00990093, solver.simplex_.lambdas(1), 1e-5);
}

TEST(GJK2DTest, gjk2D_s2d_mid) {
  cdl::np2d::GJK2D solver;
  cdl::PointType p;

  // middle case
  solver.simplex_.vertices.row(0) = Vector2r(-2.0, -2.0);
  solver.simplex_.vertices.row(1) = Vector2r(10.0, 0.0);
  solver.simplex_.vertices.row(2) = Vector2r(0, 10.0);
  solver.simplex_.num = 3;
  solver.S2D2d(solver.simplex_, p);
  EXPECT_EQ(3, solver.simplex_.num);
}

TEST(test_collision_detection_gik2d, test_Support2D)
{
    /* Find the index of the furthest point int the struct of body along the
     * search direction(v) */
    size_t ret;
    cdl::np2d::ShapeProxy2D body;
    cdl::Vector2r v;

    /* body.size = 1 */
    v = {0.0, 0.0};
    body.size = 1;
    ret = cdl::np2d::Support2D(body, v);
    EXPECT_EQ(ret, 0);

    /* body.size = 2 */
    body.size = 2;
    body.vertices[0] = {1.0, 2.0};
    body.vertices[1] = {2.0, 2.0};
    ret = cdl::np2d::Support2D(body, v);
    EXPECT_EQ(ret, 0);

    v = {1.0, 1.0};
    ret = cdl::np2d::Support2D(body, v);
    EXPECT_EQ(ret, 1);

    /* function test: initialize the parameter of body and v. */
    body.size = 3;
    body.vertices[0] = {0.0, 0.0};
    body.vertices[1] = {1.0, 0.0};
    body.vertices[2] = {1.0, 1.0};
    ret = cdl::np2d::Support2D(body, v);
    EXPECT_FLOAT_EQ(ret, 2);

    v = {1.0, 2.0};
    ret = cdl::np2d::Support2D(body, v);
    EXPECT_FLOAT_EQ(ret, 2);
}

TEST(test_collision_detection_gik2d, test_SameSign)
{
    uint8_t ret;
    cdl::real a, b;
    cdl::np2d::GJK2D gjk2d;

    a = 1.0;
    b = 1.0;
    ret = gjk2d.SameSign(a, b);
    EXPECT_EQ(ret, 1);

    a = 0.0;
    ret = gjk2d.SameSign(a, b);
    EXPECT_EQ(ret, 0);

    a = 1.0;
    b = 0.0;
    ret = gjk2d.SameSign(a, b);
    EXPECT_EQ(ret, 0);

    a = 0.0;
    ret = gjk2d.SameSign(a, b);
    EXPECT_EQ(ret, 1);
}

TEST(test_collision_detection_gik2d, test_localMinimum1)
{
    /* to judge the first point is wether a support point or not */
    bool ret;
    cdl::np2d::GJK2D gjk2d;
    cdl::np2d::ShapeProxy2D body;

    /* In the original program v={1.0, 0.0} */
    cdl::Vector2r v = {1.0, 0.0};

    Position2D P_pos[5] = {
        {3, 0},
        {4, -2},
        {5, -2},
        {5, 2},
        {4, 2}};

    body.size = 5;
    for (uint32_t i = 0; i < body.size; i++)
    {
        body.vertices[i](0) = P_pos[i].x;
        body.vertices[i](1) = P_pos[i].y;
    }

    v = {0.0, 1.0};
    ret = gjk2d.localMinimum1(body, v);
    EXPECT_TRUE(ret);

    v = {1.0, -1.0};
    ret = gjk2d.localMinimum1(body, v);
    EXPECT_FALSE(ret);

    Position2D P_pos_case2[5] = {
        {3, 0},
        {4, -1},
        {5, -1},
        {5, 2},
        {4, 2}};

    body.size = 5;
    for (uint32_t i = 0; i < body.size; i++)
    {
        body.vertices[i](0) = P_pos_case2[i].x;
        body.vertices[i](1) = P_pos_case2[i].y;
    }

    v = {1.0, 1.0};
    ret = gjk2d.localMinimum1(body, v);
    EXPECT_FALSE(ret);
}

TEST(test_collision_detection_gik2d, test_localMinimum2)
{
    bool ret;
    cdl::np2d::ShapeProxy2D P, Q;
    cdl::np2d::GJK2D gjk2d;
    cdl::Vector2r v;

    /* The first point is not the support point */
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

    v = {1, 0};

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

    ret = gjk2d.localMinimum2(P, Q, v);
    EXPECT_TRUE(ret);
    EXPECT_FLOAT_EQ(v(0), 0.0);
    EXPECT_FLOAT_EQ(v(1), 2.0);
}

TEST(test_collision_detection_gik2d, test_localMinimum)
{
    cdl::np2d::ShapeProxy2D P, Q;
    cdl::np2d::GJK2D gjk2d;
    cdl::Vector2r v = {1.0, 0.0};

    /* localMinimum1 */
    Position2D P_pos[5] = {
        {3, 0},
        {4, -2},
        {5, -2},
        {5, 2},
        {4, 2}};

    Position2D Q_pos[5] = {
        {2, 2},
        {3, -2},
        {4, 0},
        {3, 2},
        {2, 2}};

    P.size = 5;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos[i].x;
        P.vertices[i](1) = P_pos[i].y;
    }

    Q.size = 5;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos[i].x;
        Q.vertices[i](1) = Q_pos[i].y;
    }

    gjk2d.localMinimum(P, Q, v);
    EXPECT_FLOAT_EQ(v(0), 3.0);
    EXPECT_FLOAT_EQ(v(1), -4.0);

    /* localMinimum2 */
    v = {1.0, 0.0};
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

    gjk2d.localMinimum(P, Q, v);
    EXPECT_FLOAT_EQ(v(0), 0.0);
    EXPECT_FLOAT_EQ(v(1), 6.0);
}

TEST(test_collision_detection_gik2d, test_S1D2d)
{
    cdl::np2d::Simplex2D simplex;
    cdl::Vector2r v;
    cdl::np2d::GJK2D gjk2d;

    /* Initialize output parameters */
    v = {0.0, 0.0};

    /* Initialize input parameters */
    simplex.num = 2;
    simplex.vertices.row(0)(0) = 0.0;
    simplex.vertices.row(0)(1) = 1.0;
    simplex.vertices.row(1)(0) = -1.0;
    simplex.vertices.row(1)(1) = 1.0;

    gjk2d.S1D2d(simplex, v);
    EXPECT_EQ(v(0), simplex.vertices.row(0)(0));
    EXPECT_EQ(v(1), simplex.vertices.row(0)(1));

    simplex.vertices.row(0)(0) = 0.0;
    simplex.vertices.row(0)(1) = 1.0;
    simplex.vertices.row(1)(0) = 1.0;
    simplex.vertices.row(1)(1) = 1.0;

    gjk2d.S1D2d(simplex, v);
    EXPECT_EQ(v(0), simplex.vertices.row(0)(0));
    EXPECT_EQ(v(1), simplex.vertices.row(0)(1));

    /* origin in the middle case01 */
    simplex.vertices.row(0)(0) = 0.5;
    simplex.vertices.row(0)(1) = 1.0;
    simplex.vertices.row(1)(0) = -1.0;
    simplex.vertices.row(1)(1) = 1.0;
    gjk2d.S1D2d(simplex, v);

    EXPECT_FLOAT_EQ(simplex.lambdas(0), 2 / 3.0);
    EXPECT_FLOAT_EQ(simplex.lambdas(1), 1 / 3.0);
    EXPECT_FLOAT_EQ(v(1), 1.0);

    /* origin in the middle case02 */
    simplex.vertices.row(0)(0) = -1.0;
    simplex.vertices.row(0)(1) = 0.0;
    simplex.vertices.row(1)(0) = 0.0;
    simplex.vertices.row(1)(1) = 1.0;
    gjk2d.S1D2d(simplex, v);

    EXPECT_FLOAT_EQ(simplex.lambdas(0), 0.5);
    EXPECT_FLOAT_EQ(simplex.lambdas(1), 0.5);
    EXPECT_FLOAT_EQ(v(0), -0.5);
    EXPECT_FLOAT_EQ(v(1), 0.5);
}

TEST(test_collision_detection_gik2d, test_S2D2dVertexRegion)
{
    cdl::Vector2r vertex, left, right, v;
    cdl::np2d::GJK2D gjk2d;
    cdl::np2d::GJK2D::VertexRegion region;

    /* initialize simplex_, and num must be 3 */
    gjk2d.simplex_.iterations = 25;
    gjk2d.simplex_.num = 3;
    gjk2d.simplex_.vertices.row(0)(0) = -2; // c point
    gjk2d.simplex_.vertices.row(0)(1) = 3;
    gjk2d.simplex_.vertices.row(1)(0) = 0; // b point
    gjk2d.simplex_.vertices.row(1)(1) = 2;
    gjk2d.simplex_.vertices.row(2)(0) = 2; // a point
    gjk2d.simplex_.vertices.row(2)(1) = 3;

    /* initialize output parameters */
    v(0) = 0.0;
    v(1) = 0.0;

    region = cdl::np2d::GJK2D::VertexRegion::B_VERTEX;// Obtuse angle
    left = gjk2d.simplex_.vertices.row(0);
    right = gjk2d.simplex_.vertices.row(2);
    vertex = gjk2d.simplex_.vertices.row(1);

    gjk2d.S2D2dVertexRegion(vertex, left, right, region, v);
    EXPECT_EQ(gjk2d.simplex_.num, 1);
    EXPECT_FLOAT_EQ(gjk2d.simplex_.lambdas(0), 1);
    EXPECT_FLOAT_EQ(v(0), 0.0);
    EXPECT_FLOAT_EQ(v(1), 2.0);

    gjk2d.simplex_.iterations = 25;
    gjk2d.simplex_.num = 3;
    gjk2d.simplex_.vertices.row(0)(0) = 5;
    gjk2d.simplex_.vertices.row(0)(1) = 0;
    gjk2d.simplex_.vertices.row(1)(0) = 1;
    gjk2d.simplex_.vertices.row(1)(1) = 0;
    gjk2d.simplex_.vertices.row(2)(0) = 3;
    gjk2d.simplex_.vertices.row(2)(1) = -1;
    region = cdl::np2d::GJK2D::VertexRegion::B_VERTEX;// Acute angle
    left = gjk2d.simplex_.vertices.row(2);
    right = gjk2d.simplex_.vertices.row(0);
    vertex = gjk2d.simplex_.vertices.row(1);
    gjk2d.S2D2dVertexRegion(vertex, left, right, region, v);
    EXPECT_EQ(gjk2d.simplex_.num, 1);
    EXPECT_FLOAT_EQ(gjk2d.simplex_.lambdas(0), 1);

    /* v and b point are the same */
    EXPECT_FLOAT_EQ(v(0), 1.0);
    EXPECT_FLOAT_EQ(v(1), 0.0);
}

TEST(test_collision_detection_gik2d, test_S2D2d)
{
    cdl::np2d::Simplex2D simplex;
    cdl::Vector2r v;
    cdl::np2d::GJK2D gjk2d;

    /* initialize simplex_, and num must be 3 */
    simplex.iterations = 25;
    simplex.num = 3;

    /* initialize output parameters */
    v(0) = 0.0;
    v(1) = 0.0;

    /* case7: */
    simplex.vertices.row(0)(0) = 2;
    simplex.vertices.row(0)(1) = 0;
    simplex.vertices.row(1)(0) = 0;
    simplex.vertices.row(1)(1) = 1;
    simplex.vertices.row(2)(0) = -2;
    simplex.vertices.row(2)(1) = 0;

    gjk2d.S2D2d(simplex, v);
    EXPECT_EQ(simplex.num, 3);
    EXPECT_FLOAT_EQ(v(0), 0.0);
    EXPECT_FLOAT_EQ(v(1), 0.0);

    /* case4: */
    simplex.vertices.row(0)(0) = 2;
    simplex.vertices.row(0)(1) = 2;
    simplex.vertices.row(1)(0) = 0;
    simplex.vertices.row(1)(1) = 2;
    simplex.vertices.row(2)(0) = 1;
    simplex.vertices.row(2)(1) = 1.5;
    gjk2d.S2D2d(simplex, v);
    EXPECT_EQ(simplex.num, 3);
    EXPECT_FLOAT_EQ(v(0), 0.0);
    EXPECT_FLOAT_EQ(v(1), 0.0);

    /* case6: */
    simplex.vertices.row(0)(0) = 1;
    simplex.vertices.row(0)(1) = 3;
    simplex.vertices.row(1)(0) = 0.5;
    simplex.vertices.row(1)(1) = 2;
    simplex.vertices.row(2)(0) = 1;
    simplex.vertices.row(2)(1) = 1;
    gjk2d.S2D2d(simplex, v);
    EXPECT_EQ(simplex.num, 1);

    /* case5: */
    simplex.vertices.row(0)(0) = 0.123605;
    simplex.vertices.row(0)(1) = -3.264827;
    simplex.vertices.row(1)(0) = 1.022984;
    simplex.vertices.row(1)(1) = 0.699159;
    simplex.vertices.row(2)(0) = 0.113750;
    simplex.vertices.row(2)(1) = 0.124095;

    gjk2d.S2D2d(simplex, v);
    EXPECT_EQ(simplex.num, 2);

}

TEST(test_collision_detection_gik2d, test_Collision)
{
    bool ret;
    cdl::np2d::GJK2D gjk_solver;
    cdl::np2d::ShapeProxy2D P, Q;

    /* no collision */
    Position2D P_pos[4] = {
        {2, 2},
        {1, 3},
        {0, 2},
        {1, 1}};

    Position2D Q_pos[3] = {
        {4, 0.5},
        {2, 1.5},
        {2, 0.5}};

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos[i].x;
        P.vertices[i](1) = P_pos[i].y;
    }
    Q.size = 3;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos[i].x;
        Q.vertices[i](1) = Q_pos[i].y;
    }
    ret = gjk_solver.Collision(P, Q);
    EXPECT_EQ(ret, false);

    /* Just one point collision */
    Position2D P_pos_case2[4] = {
        {2, 2},
        {1, 3},
        {0, 2},
        {1, 1}};

    Position2D Q_pos_case2[3] = {
        {4, 0.5},
        {2, 2},
        {2, 0.5},
    };

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case2[i].x;
        P.vertices[i](1) = P_pos_case2[i].y;
    }
    Q.size = 3;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case2[i].x;
        Q.vertices[i](1) = Q_pos_case2[i].y;
    }
    ret = gjk_solver.Collision(P, Q);
    EXPECT_EQ(ret, true);

    /* Face collision */
    Position2D P_pos_case3[4] = {
        {2, 2},
        {1, 3},
        {0, 2},
        {1, 1}};

    Position2D Q_pos_case3[3] = {
        {4, 0.5},
        {1, 2},
        {2, 0.5},
    };

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case3[i].x;
        P.vertices[i](1) = P_pos_case3[i].y;
    }
    Q.size = 3;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case3[i].x;
        Q.vertices[i](1) = Q_pos_case3[i].y;
    }
    ret = gjk_solver.Collision(P, Q);
    EXPECT_EQ(ret, true);

    /* Can't resolve the near collision */
    Position2D P_pos_case4[4] = {
        {2, 2},
        {1, 3},
        {0, 2},
        {1, 1}};

    Position2D Q_pos_case4[3] = {
        {4, 0.5},
        {2 + 0.1 * gjk_solver.eps_tol, 2},
        {2, 0.5},
    };

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case4[i].x;
        P.vertices[i](1) = P_pos_case4[i].y;
    }
    Q.size = 3;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case4[i].x;
        Q.vertices[i](1) = Q_pos_case4[i].y;
    }
    ret = gjk_solver.Collision(P, Q);
    EXPECT_EQ(ret, false);
}

TEST(test_collision_detection_gik2d, test_collision_test02)
{
    bool ret;
    cdl::np2d::GJK2D gjk_solver;
    cdl::np2d::ShapeProxy2D P, Q;



    /* no collision */
    Position2D P_pos[4] = {
        {-77171.33027, 4389770.40190},
        {-77172.81957, 4389768.44394},
        {-77169.13846, 4389765.64394},
        {-77167.64916, 4389767.60189}};

    Position2D Q_pos[7] = {
        {-77172.88329, 4389771.01634},
        {-77172.95072, 4389769.96127},
        {-77172.80062, 4389769.77367},

        { -77172.29165, 4389769.39477 },
        { -77171.50128, 4389769.56122 },
        { -77171.05531, 4389769.85848 },
        {-77171.09032, 4389769.93234}};

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos[i].x;
        P.vertices[i](1) = P_pos[i].y;
    }
    Q.size = 7;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos[i].x;
        Q.vertices[i](1) = Q_pos[i].y;
    }
    ret = gjk_solver.Collision(P, Q);
    printf("collision \n");
    EXPECT_EQ(ret, true);

    Position2D P_pos_case02[4] = {
        {1, 0},
        {0, 2},
        {-1, 0},
        {0, -2}};

    Position2D Q_pos_case02[4] = {
        {2, 2},
        {1, 4},
        {0, 2},
        {1, 0}};

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case02[i].x;
        P.vertices[i](1) = P_pos_case02[i].y;
    }
    Q.size = 4;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case02[i].x;
        Q.vertices[i](1) = Q_pos_case02[i].y;
    }
    ret = gjk_solver.Collision(P, Q);
    EXPECT_EQ(ret, true);

    Position2D P_pos_case03[3] = {
        {0, 0},
        {0, 4},
        {-6, 0}};

    Position2D Q_pos_case03[4] = {
        {-5, -2},
        {-5, 2},
        {-9, 2},
        {-9, -2}};

    P.size = 3;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case03[i].x;
        P.vertices[i](1) = P_pos_case03[i].y;
    }
    Q.size = 4;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case03[i].x;
        Q.vertices[i](1) = Q_pos_case03[i].y;
    }
    ret = gjk_solver.Collision(P, Q);
    EXPECT_EQ(ret, true);
}

TEST(test_collision_detection_gik2d, test_Distance)
{
    cdl::np2d::GJK2D gjk_solver;
    cdl::np2d::DistResult2D gjk2d_result;
    cdl::np2d::ShapeProxy2D P, Q;

    /* face collision */
    Position2D P_pos[4] = {
        {2, 2},
        {1, 3},
        {0, 2},
        {1, 1}};

    Position2D Q_pos[3] = {
        {4, 0.5},
        {2, 1.5},
        {2, 0.5},
    };

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos[i].x;
        P.vertices[i](1) = P_pos[i].y;
    }
    Q.size = 3;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos[i].x;
        Q.vertices[i](1) = Q_pos[i].y;
    }

    double error = 0.002;
    gjk2d_result.p = cdl::Vector2r::Zero();
    gjk2d_result.q = cdl::Vector2r::Zero();
    gjk_solver.Distance(P, Q, &gjk2d_result);
    EXPECT_NEAR(1.0 / apollo_sqrt(2) / 2.0, gjk2d_result.distance, error);

    /* Just one point collision: case01 */
    Position2D P_pos_case2[4] = {
        {2, 2},
        {1, 3},
        {0, 2},
        {1, 1}
    };

    Position2D Q_pos_case2[3] = {
        {4, 0.5},
        {2, 2},
        {2, 0.5},
    };

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case2[i].x;
        P.vertices[i](1) = P_pos_case2[i].y;
    }
    Q.size = 3;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case2[i].x;
        Q.vertices[i](1) = Q_pos_case2[i].y;
    }
    gjk_solver.Distance2(P, Q, &gjk2d_result);
    EXPECT_NEAR(0, gjk2d_result.distance, error);

    gjk_solver.Distance(P, Q, &gjk2d_result);
    EXPECT_NEAR(0, gjk2d_result.distance, error);

    /* Just one point collision: case02 */
    Position2D P_pos_case2_02[4] = {
        {2, 2},
        {1, 3},
        {0, 2},
        {1, 1}};

    Position2D Q_pos_case2_02[2] = {
        {0, 2},
        {2, 2}
    };

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case2_02[i].x;
        P.vertices[i](1) = P_pos_case2_02[i].y;
    }
    Q.size = 1;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case2_02[i].x;
        Q.vertices[i](1) = Q_pos_case2_02[i].y;
    }
    gjk_solver.Distance2(P, Q, &gjk2d_result);
    EXPECT_NEAR(0, gjk2d_result.distance, error);

    gjk_solver.Distance(P, Q, &gjk2d_result);
    EXPECT_NEAR(0, gjk2d_result.distance, error);

    /* Face collision */
    Position2D P_pos_case3[4] = {
        {2, 2},
        {1, 3},
        {0, 2},
        {1, 1}};

    Position2D Q_pos_case3[3] = {
        {4, 0.5},
        {1, 2},
        {2, 0.5},
    };

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case3[i].x;
        P.vertices[i](1) = P_pos_case3[i].y;
    }
    Q.size = 3;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case3[i].x;
        Q.vertices[i](1) = Q_pos_case3[i].y;
    }
    gjk_solver.Distance2(P, Q, &gjk2d_result);
    EXPECT_NEAR(0, gjk2d_result.distance, error);

    gjk_solver.Distance(P, Q, &gjk2d_result);
    EXPECT_NEAR(0, gjk2d_result.distance, error);

    /* Minimal distance */
    Position2D P_pos_case4[4] = {
        {2, 2},
        {1, 3},
        {0, 2},
        {1, 1}};

    Position2D Q_pos_case4[3] = {
        {4, 0.5},
        {2 + 0.1 * gjk_solver.eps_tol, 2},
        {2, 0.5},
    };

    P.size = 4;
    for (uint32_t i = 0; i < P.size; i++)
    {
        P.vertices[i](0) = P_pos_case4[i].x;
        P.vertices[i](1) = P_pos_case4[i].y;
    }
    Q.size = 3;
    for (uint32_t i = 0; i < Q.size; i++)
    {
        Q.vertices[i](0) = Q_pos_case4[i].x;
        Q.vertices[i](1) = Q_pos_case4[i].y;
    }
    gjk_solver.Distance2(P, Q, &gjk2d_result);
    EXPECT_NEAR(0, gjk2d_result.distance, error);
}




