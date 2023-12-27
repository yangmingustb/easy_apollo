#include "cdl/narrowphase/gjk/gjk2d.h"
#include <gtest/gtest.h>
#include <viz/viewer.h>
#include <iostream>
#include <string>
#include "cdl/cdl.h"
#include "cdl/common/math.h"
#include "cdl/data_generator.h"

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
#define PI 3.1415926

TEST(RAYCAST, test1)
{
    for (size_t i = 0; i < 10; i++)
    {
        Status object_status = Status::Separated;
        DataGenerator tf_data(object_status);
        
        std::vector<cdl::Vector3r> vertices8;
        int num = 12;
        for (size_t i = 0; i < num; i++)
        {
            double x = cos(2 * PI / num * i);
            double y = sin(2 * PI / num * i);
            vertices8.push_back(cdl::Vector3r(x, y, 0.0));
        }

        std::vector<cdl::PointType> vertices;
        for (size_t i = 0; i < vertices8.size(); i++)
        {
            cdl::Vector3r data = tf_data.rect_tf.linear() * vertices8[i];
            vertices.push_back(data.head(2));
        }
        cdl::np2d::RayCastRequest input;
        input.source = cdl::Vector2r(0, 10.0);
        input.direction = cdl::Vector2r(0, -1.03);
        input.max_lambda = 20;
        input.shape.vertices = &vertices;

        // set output
        cdl::np2d::RayCastResult result;

        cdl::np2d::GJK2D gjk_solver;
        gjk_solver.GJKBasedRayCast(&input, &result);

        cdl::Vector2r flag = 
            result.hit_spot - (input.source + input.direction * result.lambda_hit);
        
        if ( cdl::Dot2D(flag, flag) > 0.0001)
        {
            std::cout<<"------------------- "<<std::endl;
            printf("not match between hit spot and hit lambda \n");
            std::cout<<"hit spot "<<result.hit_spot<<std::endl;
            std::cout<<"hit lambda "<<input.source + input.direction * result.lambda_hit
            <<std::endl;

        }
        printStatus(result);

        std::vector<cdl::Vector2r> ray;
        ray.reserve(2);
        ray.push_back(input.source);
        ray.push_back(input.source + input.max_lambda * input.direction);
        cdl::np2d::ShapeProxy2D P;
        P.vertices = &ray;
        drawObjectAndCollision(P, input.shape, result);
        
    }
}

// HIT
TEST(GJKBASED_RAYCAST, HIT_rng)
{
    for (size_t i = 0; i < 10; i++)
    {
        std::cout << "gjk based raycast. random test "<< i << std::endl;
        Status object_status = Status::Separated;
        DataGenerator object_data(object_status);

        std::vector<cdl::PointType> vertices;
        for (size_t i = 0; i < object_data.rect_vertices.size(); i++)
        {
            cdl::Vector3r object1 =
                object_data.rect_tf.linear() * object_data.rect_vertices[i];
            vertices.push_back(object1.head(2));
        }
        cdl::np2d::RayCastRequest input;
        input.source = cdl::Vector2r(0.0, 30.5);
        input.direction = cdl::Vector2r(0.0, -1.02);
        input.max_lambda = 50;
        input.shape.vertices = &vertices;

        // set output
        cdl::np2d::RayCastResult result;

        cdl::np2d::GJK2D gjk_solver;
        gjk_solver.GJKBasedRayCast(&input, &result);
        std::cout << "distance:" << result.hit_distance << std::endl;
        std::cout << "normal:" << result.hit_normal << std::endl;
        std::cout << "hit spot:" << result.hit_spot << std::endl;
        std::cout << "lambda hit:" << result.lambda_hit << std::endl;
        cdl::Vector2r flag = 
            result.hit_spot - (input.source + input.direction * result.lambda_hit);
        
        if ( cdl::Dot2D(flag, flag) > 0.0001)
        {
            std::cout<<"------------------- "<<std::endl;
            printf("not match between hit spot and hit lambda \n");
            std::cout<<"hit spot "<<result.hit_spot<<std::endl;
            std::cout<<"hit lambda "<<input.source + input.direction * result.lambda_hit
            <<std::endl;

        }
        printStatus(result);

        std::vector<cdl::Vector2r> ray;
        ray.reserve(2);
        ray.push_back(input.source);
        ray.push_back(input.source + input.max_lambda * input.direction);
        cdl::np2d::ShapeProxy2D P;
        P.vertices = &ray;
        drawObjectAndCollision(P, input.shape, result);
    }

    for (size_t i = 0; i < 10; i++)
    {
        std::cout << "gjk based raycast. random test "<< i << std::endl;
        Status object_status = Status::Separated;
        DataGenerator object_data(object_status);

        std::vector<cdl::PointType> vertices;
        for (size_t i = 0; i < object_data.panta_vertices.size(); i++)
        {
            cdl::Vector3r object1 =
                object_data.panta_tf.linear() * object_data.panta_vertices[i];
            vertices.push_back(object1.head(2));
        }
        cdl::np2d::RayCastRequest input;
        input.source = cdl::Vector2r(-30.0, 0.5);
        input.direction = cdl::Vector2r(1.0, 0.2);
        input.max_lambda = 50;
        input.shape.vertices = &vertices;

        // set output
        cdl::np2d::RayCastResult result;

        cdl::np2d::GJK2D gjk_solver;
        gjk_solver.GJKBasedRayCast(&input, &result);
        std::cout << "distance:" << result.hit_distance << std::endl;
        std::cout << "normal:" << result.hit_normal << std::endl;
        std::cout << "hit spot:" << result.hit_spot << std::endl;
        std::cout << "lambda hit:" << result.lambda_hit << std::endl;
        cdl::Vector2r flag = 
            result.hit_spot - (input.source + input.direction * result.lambda_hit);
        
        if ( cdl::Dot2D(flag, flag) > 0.00001)
        {
            std::cout<<"------------------- "<<std::endl;
            printf("not match between hit spot and hit lambda \n");
            std::cout<<"hit spot "<<result.hit_spot<<std::endl;
            std::cout<<"hit lambda "<<input.source + input.direction * result.lambda_hit
            <<std::endl;

        }
        

        printStatus(result);

        std::vector<cdl::Vector2r> ray;
        ray.reserve(2);
        ray.push_back(input.source);
        ray.push_back(input.source + input.max_lambda * input.direction);
        cdl::np2d::ShapeProxy2D P;
        P.vertices = &ray;
        drawObjectAndCollision(P, input.shape, result);
    }
}
// HIT
TEST(GJKBASED_RAYCAST, HIT1)
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
    gjk_solver.GJKBasedRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;
    int flag;
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            flag = 2;
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            flag = 0;
            break;
        case cdl::np2d::RayCastStatus::HIT:
            flag = 1;
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            flag = 3;
            break;
        default:
            break;
    }
    std::cout << "status:" << flag << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

TEST(GJKBASED_RAYCAST, HIT_debug1)
{
    std::cout << "gjk based raycast! one loop!" << std::endl;
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(2.0, 0.0));
    vertices.push_back(Vector2r(5.0, 1.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 3.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(-10, 1);
    input.direction = cdl::Vector2r(1, 0);
    input.max_lambda = 30;
    input.shape.vertices = &vertices;

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.GJKBasedRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;
    int flag;
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            flag = 2;
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            flag = 0;
            break;
        case cdl::np2d::RayCastStatus::HIT:
            flag = 1;
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            flag = 3;
            break;
        default:
            break;
    }
    std::cout << "status:" << flag << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// penetration
TEST(GJKBASED_RAYCAST, internal_source)
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
    input.max_lambda = 5;
    input.shape.vertices = &vertices;

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.GJKBasedRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;
    int flag;
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            flag = 2;
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            flag = 0;
            break;
        case cdl::np2d::RayCastStatus::HIT:
            flag = 1;
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            flag = 3;
            break;
        default:
            break;
    }
    std::cout << "status:" << flag << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// HIT
TEST(GJKBASED_RAYCAST, internal_source2)
{
    std::cout << "gjk based raycast! one loop!" << std::endl;
    std::vector<cdl::PointType> vertices;
    vertices.push_back(Vector2r(0.0, 0.0));
    vertices.push_back(Vector2r(5.0, 0.0));
    vertices.push_back(Vector2r(5.0, 5.0));
    vertices.push_back(Vector2r(0.0, 5.0));

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(2, 1);
    input.direction = cdl::Vector2r(1, 0);
    input.max_lambda = 30;
    input.shape.vertices = &vertices;

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.GJKBasedRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;
    int flag;
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            flag = 2;
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            flag = 0;
            break;
        case cdl::np2d::RayCastStatus::HIT:
            flag = 1;
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            flag = 3;
            break;

        default:
            break;
    }
    std::cout << "status:" << flag << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

#define PI 3.1415926

TEST(GJKBASED_RAYCAST, internal_source3)
{
    std::cout << "gjk based raycast! one loop!" << std::endl;
    std::vector<cdl::PointType> vertices;
    for (size_t i = 0; i < 20; i++)
    {
        double x = cos(2 * PI / 20.0 * i);
        double y = sin(2 * PI / 20.0 * i);
        vertices.push_back(Vector2r(x + 1, y + 1));
    }

    // set input
    cdl::np2d::RayCastRequest input;
    input.source = cdl::Vector2r(1, 1);
    input.direction = cdl::Vector2r(1, 0);
    input.max_lambda = 30;
    input.shape.vertices = &vertices;

    // set output
    cdl::np2d::RayCastResult result;

    cdl::np2d::GJK2D gjk_solver;
    gjk_solver.GJKBasedRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;
    int flag;
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            flag = 2;
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            flag = 0;
            break;
        case cdl::np2d::RayCastStatus::HIT:
            flag = 1;
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            flag = 3;
            break;
        default:
            break;
    }
    std::cout << "status:" << flag << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// HIT
TEST(GJKBASED_RAYCAST, HIT2)
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
    gjk_solver.GJKBasedRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;
    int flag;
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            flag = 2;
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            flag = 0;
            break;
        case cdl::np2d::RayCastStatus::HIT:
            flag = 1;
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            flag = 3;
            break;
        default:
            break;
    }
    std::cout << "status:" << flag << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// HIT
TEST(GJKBASED_RAYCAST, HIT3)
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
    gjk_solver.GJKBasedRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;
    int flag;
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            flag = 2;
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            flag = 0;
            break;
        case cdl::np2d::RayCastStatus::HIT:
            flag = 1;
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            flag = 3;
            break;
        default:
            break;
    }
    std::cout << "status:" << flag << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// HIT
TEST(GJKBASED_RAYCAST, HIT4)
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
    gjk_solver.GJKBasedRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;
    int flag;
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            flag = 2;
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            flag = 0;
            break;
        case cdl::np2d::RayCastStatus::HIT:
            flag = 1;
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            flag = 3;
            break;
        default:
            break;
    }
    std::cout << "status:" << flag << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// HIT
TEST(GJKBASED_RAYCAST, HIT5)
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
    gjk_solver.GJKBasedRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;
    int flag;
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            flag = 2;
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            flag = 0;
            break;
        case cdl::np2d::RayCastStatus::HIT:
            flag = 1;
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            flag = 3;
            break;
        default:
            break;
    }
    std::cout << "status:" << flag << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// HIT
TEST(GJKBASED_RAYCAST, HIT6)
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
    gjk_solver.GJKBasedRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;
    int flag;
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            flag = 2;
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            flag = 0;
            break;
        case cdl::np2d::RayCastStatus::HIT:
            flag = 1;
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            flag = 3;
            break;
        default:
            break;
    }
    std::cout << "status:" << flag << std::endl;
    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// HIT
TEST(NESTED_LOOP_RAYCAST, HIT10)
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
    gjk_solver.GJKBasedRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;
    int flag;
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            flag = 2;
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            flag = 0;
            break;
        case cdl::np2d::RayCastStatus::HIT:
            flag = 1;
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            flag = 3;
            break;
        default:
            break;
    }
    std::cout << "status:" << flag << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}

// HIT
TEST(NESTED_LOOP_RAYCAST, HIT2)
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
    gjk_solver.GJKBasedRayCast(&input, &result);
    std::cout << "distance:" << result.hit_distance << std::endl;
    std::cout << "normal:" << result.hit_normal << std::endl;
    std::cout << "hit spot:" << result.hit_spot << std::endl;
    std::cout << "lambda hit:" << result.lambda_hit << std::endl;
    int flag;
    switch (result.status)
    {
        case cdl::np2d::RayCastStatus::SOURCE_IS_INSIDE:
            flag = 2;
            break;
        case cdl::np2d::RayCastStatus::NO_HIT:
            flag = 0;
            break;
        case cdl::np2d::RayCastStatus::HIT:
            flag = 1;
            break;
        case cdl::np2d::RayCastStatus::WILL_HIT:
            flag = 3;
            break;
        default:
            break;
    }
    std::cout << "status:" << flag << std::endl;

    std::vector<cdl::Vector2r> ray;
    ray.reserve(2);
    ray.push_back(input.source);
    ray.push_back(input.source + input.max_lambda * input.direction);
    cdl::np2d::ShapeProxy2D P;
    P.vertices = &ray;
    drawObjectAndCollision(P, input.shape, result);
}
