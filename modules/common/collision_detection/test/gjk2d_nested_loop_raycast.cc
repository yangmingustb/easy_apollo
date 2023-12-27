#include "cdl/narrowphase/gjk/gjk2d.h"
#include <gtest/gtest.h>
#include <viz/viewer.h>
#include <iostream>
#include <string>
#include "cdl/common/math.h"

using cdl::Vector2r;

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

void dispalyRay(osg::Vec3 origin, osg::Vec3 endpoint, Viewer *viz,
                osg::Transform *tf, const osg::Vec4 &color)
{
    viz->createLine(tf, origin, endpoint, color, 2.0);
    viz->createSphere(tf, origin, 0.1, color);
}

void displayCollisionObject(osg::Vec3Array *polygon2, Viewer *viz,
                            osg::Transform *tf)
{
    viz->createLineLoop(tf, polygon2, viz->black, 1);
}

void drawObjectAndCollision(cdl::np2d::ShapeProxy2D &shape1,
                            cdl::np2d::ShapeProxy2D &shape2,
                            cdl::np2d::RayCastResult &result)
{
    int number1 = shape1.vertices->size();
    int number2 = shape2.vertices->size();
    osg::Vec3Array *polygonA = new osg::Vec3Array(number1);
    osg::Vec3Array *polygonB = new osg::Vec3Array(number2);
    for (int i = 0; i < number1; i++)
    {
        osg::Vec3 p(shape1.vertices->at(i)[0], shape1.vertices->at(i)[1], 0);
        polygonA->at(i) = p;
    }
    for (int i = 0; i < number2; i++)
    {
        osg::Vec3 p(shape2.vertices->at(i)[0], shape2.vertices->at(i)[1], 0);
        polygonB->at(i) = p;
    }
    Viewer viz;
    osg::ref_ptr<osg::MatrixTransform> osg_tf = new osg::MatrixTransform;
    osg::Vec3 hit_spot, normalA, normalB;
    hit_spot.set(result.hit_spot(0), result.hit_spot(1), 0);

    normalA.set(result.hit_spot(0), result.hit_spot(1), 1);
    cdl::Vector2r normal_end = result.hit_spot + result.hit_normal;
    normalB.set(normal_end(0), normal_end(1), 1);

    displayCollisionObject(polygonB, &viz, osg_tf);

    dispalyRay(polygonA->at(0), polygonA->at(1), &viz, osg_tf, viz.purple);

    if (result.status == cdl::np2d::RayCastStatus::HIT)
    {
        // display hit spot
        viz.createSphere(osg_tf, hit_spot, 0.1, viz.yellow);
        // display hit spot edge normal
        dispalyRay(normalA, normalB, &viz, osg_tf, viz.green);
    }
    viz.setWindowName("epa 2d gtest");
    viz.setDisplayData();
    viz.viewer->run();
}

// penetration
TEST(GJKBASED_RAYCAST, penetration1)
{
    std::cout << "gjk based raycast! one loop!" << std::endl;
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
    input.shape.vertices = &vertices;

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.nestedLoopRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// penetration
TEST(GJKBASED_RAYCAST, penetration2)
{
    std::cout << "gjk based raycast! one loop!" << std::endl;
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
    input.shape.vertices = &vertices;

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.nestedLoopRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// penetration
TEST(GJKBASED_RAYCAST, penetration3)
{
    std::cout << "gjk based raycast! one loop!" << std::endl;
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, 0.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 5.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(-10, -10);
    input.direction = cdl::Vector2r(1, 1);
    input.max_lambda = 30;
    input.shape.vertices = &vertices;

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.nestedLoopRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// penetration
TEST(GJKBASED_RAYCAST, penetration4)
{
    std::cout << "gjk based raycast! one loop!" << std::endl;
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, 0.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 5.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(-10, -10);
    input.direction = cdl::Vector2r(1, 0.9);
    input.max_lambda = 30;
    input.shape.vertices = &vertices;

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.nestedLoopRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// penetration
TEST(GJKBASED_RAYCAST, penetration5)
{
    std::cout << "gjk based raycast! one loop!" << std::endl;
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, 0.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 5.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(-10, -10);
    input.direction = cdl::Vector2r(1, 0.7);
    input.max_lambda = 30;
    input.shape.vertices = &vertices;

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.nestedLoopRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// penetration
TEST(GJKBASED_RAYCAST, penetration6)
{
    std::cout << "gjk based raycast! one loop!" << std::endl;
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, 0.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 5.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(-10, -10);
    input.direction = cdl::Vector2r(1, 0.6);
    input.max_lambda = 30;
    input.shape.vertices = &vertices;

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.nestedLoopRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// penetration
TEST(NESTED_LOOP_RAYCAST, penetration10)
{
    std::cout << "nested loop gjk raycast test!" << std::endl;
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
    input.shape.vertices = &vertices;

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.nestedLoopRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// penetration
TEST(NESTED_LOOP_RAYCAST, penetration2)
{
    std::cout << "nested loop gjk raycast test!" << std::endl;
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, -2.0));
    vertices.push_back(Vector2r(8.0, 0.0));
    vertices.push_back(Vector2r(10.0, 3.0));
    vertices.push_back(Vector2r(8.0, 5.0));
    vertices.push_back(Vector2r(-1.0, 1.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(-10, 1);
    input.direction = cdl::Vector2r(1, 0.1);
    input.max_lambda = 30;
    input.shape.vertices = &vertices;

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.nestedLoopRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}