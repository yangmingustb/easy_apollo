#include <cdl/cdl.h>
#include <gtest/gtest.h>
#include <viz/viewer.h>
#include <unordered_set>
#include "cdl_c/hierarchy_collision_wrapper.h"
#include "cdl_c/uos_utils.h"

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

cdl::RNG yaw_rng;

cdl::aligned_vector<cdl::Transform3r> getTrajectory(cdl::real x, cdl::real y,
                                                    cdl::real yaw,
                                                    std::size_t size,
                                                    cdl::real delta_length,
                                                    cdl::real delta_yaw)
{
    cdl::aligned_vector<cdl::Transform3r> trajectory;
    trajectory.reserve(size);
    for (std::size_t i = 0; i < size; ++i)
    {
        cdl::Transform3r tf;
        tf.setIdentity();
        if (i == 0)
        {
            cdl::setYaw(tf, yaw);
            tf.translation() << x, y, 0;
        }
        else
        {
            cdl::real random_delta_yaw =
                std::copysign(delta_yaw, yaw_rng.uniformReal(-1, 1));
            cdl::updatePose(tf, trajectory.back(), delta_length,
                            random_delta_yaw);
        }
        trajectory.emplace_back(std::move(tf));
    }
    return trajectory;
}

void drawFootprint(const cdl::Transform3r& tf, const cdl::Convex* poly,
                   Viewer* viz, const osg::Vec4& color, cdl::real line_width)
{
    osg::ref_ptr<osg::MatrixTransform> osg_tf =
        new osg::MatrixTransform(osg::Matrix::translate(0.0, 0.0, 0.0));
    // convert local to global frame
    auto vertices = poly->getBoundVertices(tf);
    osg::Vec3Array* osg_vertices = new osg::Vec3Array(vertices.size());
    for (int i = 0; i < vertices.size(); ++i)
    {
        osg_vertices->at(i)[0] = vertices.at(i)[0];
        osg_vertices->at(i)[1] = vertices.at(i)[1];
        osg_vertices->at(i)[2] = vertices.at(i)[2];
    }
    viz->createLineLoop(osg_tf, osg_vertices, color, line_width);
}

void tagFootprint(const cdl::Transform3r& tf, const cdl::Convex* poly,
                  Viewer* viz, const osg::Vec4& color, cdl::real line_width)
{
    osg::ref_ptr<osg::MatrixTransform> osg_tf =
        new osg::MatrixTransform(osg::Matrix::translate(0.0, 0.0, 0.0));
    // convert local to global frame
    auto vertices = poly->getBoundVertices(tf);
    osg::Vec3Array* osg_vertices = new osg::Vec3Array(vertices.size());
    for (int i = 0; i < vertices.size(); ++i)
    {
        osg_vertices->at(i)[0] = vertices.at(i)[0];
        osg_vertices->at(i)[1] = vertices.at(i)[1];
        osg_vertices->at(i)[2] = vertices.at(i)[2];
        viz->createSphere(osg_tf, osg_vertices->at(i), 0.1, viz->white);
    }
    viz->createPolygon(osg_tf, osg_vertices, color, line_width);
}

TEST(CDL, filter_test)
{
    // pure daabb test with filter enabled
    auto footprint = getVehicleFootPrint(3.0, 1.0, 1.0, 1.0);
    auto ego_traj = getTrajectory(2, 2, 0.5, 50, 2.0, 0.1);
    auto mot_traj = getTrajectory(20.0, 2.0, 2.0, 50, 2.0, 0.2);
    int ego_traj_id = 0;
    int mot_traj_id = 1;

    Viewer viz;
    for (const auto& tf : ego_traj)
    {
        drawFootprint(tf, footprint.get(), &viz, viz.blue, 1.0);
    }

    for (const auto& tf : mot_traj)
    {
        drawFootprint(tf, footprint.get(), &viz, viz.purple, 1.0);
    }

    // aabb manager setup
    std::vector<cdl::CollisionObject*> ego_cos_vec;
    std::vector<cdl::CollisionObject*> mot_cos_vec;
    std::vector<cdl::UserData> ego_ud_vec;
    std::vector<cdl::UserData> mot_ud_vec;
    ego_cos_vec.reserve(ego_traj.size());
    mot_cos_vec.reserve(mot_traj.size());
    ego_ud_vec.reserve(ego_traj.size());
    mot_ud_vec.reserve(mot_traj.size());

    for (std::size_t i = 0; i < ego_traj.size(); ++i)
    {
        auto& tf = ego_traj.at(i);
        auto mot_co = new cdl::CollisionObject(footprint, tf);
        cdl::UserData ud;
        ud.id = i;
        ud.traj_id = ego_traj_id;
        ud.pose_id = i;
        ego_cos_vec.emplace_back(mot_co);
        ego_ud_vec.emplace_back(std::move(ud));
        ego_cos_vec.back()->setUserData(&ego_ud_vec.back());
    }

    for (std::size_t i = 0; i < mot_traj.size(); ++i)
    {
        auto& tf = mot_traj.at(i);
        auto mot_co = new cdl::CollisionObject(footprint, tf);
        cdl::UserData ud;
        ud.id = i;
        ud.traj_id = mot_traj_id;
        ud.pose_id = i;
        mot_cos_vec.emplace_back(mot_co);
        mot_ud_vec.emplace_back(std::move(ud));
        mot_cos_vec.back()->setUserData(&mot_ud_vec.back());
    }

    cdl::DynamicAABBTreeCollisionManager ego_manager;
    cdl::DynamicAABBTreeCollisionManager mot_manager;
    ego_manager.registerObjects(ego_cos_vec);
    ego_manager.setup();
    mot_manager.registerObjects(mot_cos_vec);
    mot_manager.setup();

    cdl::CollisionData collision_data;
    collision_data.request.num_max_contacts = 10000;
    collision_data.request.enable_contact = true;
    collision_data.request.enable_filter = false;
    collision_data.request.filter_set.insert(mot_traj_id);

    ego_manager.collide(&mot_manager, &collision_data, cdl::CollisionFunction);

    std::cout << "tree vs tree contact num:"
              << collision_data.result.numContacts() << std::endl;
    if (collision_data.result.numContacts() > 0)
    {
        std::vector<cdl::Contact> collisions;
        collision_data.result.getContacts(collisions);
        std::stable_sort(collisions.begin(), collisions.end(), cdl::compare2ID);
        for (const auto& ct : collisions)
        {
            std::cout << "ct b1:" << ct.b1 << ", b2:" << ct.b2 << std::endl;
        }
    }

    // enable filter, but with an inactive filter number, check the contact
    // number,
    collision_data.request.num_max_contacts = 10000;
    collision_data.request.enable_contact = true;
    collision_data.request.enable_filter = true;
    collision_data.request.filter_set.clear();
    collision_data.request.filter_set.insert(mot_traj_id);
    collision_data.result.clear();

    mot_manager.collide(ego_cos_vec.at(8), &collision_data,
                        cdl::CollisionFunction);
    std::cout << "enable filter, only check trajectory tagged by filter"
              << std::endl;
    std::cout << "pose vs tree contact num:"
              << collision_data.result.numContacts() << std::endl;
    if (collision_data.result.numContacts() > 0)
    {
        std::vector<cdl::Contact> collisions;
        collision_data.result.getContacts(collisions);
        std::stable_sort(collisions.begin(), collisions.end(), cdl::compare2ID);
        for (const auto& ct : collisions)
        {
            std::cout << "ct b1:" << ct.b2 << ", b2:" << ct.b1 << std::endl;
        }
    }

    // enable filter, check the contact number
    collision_data.request.num_max_contacts = 10000;
    collision_data.request.enable_contact = true;
    collision_data.request.enable_filter = true;
    collision_data.request.filter_set.clear();
    collision_data.request.filter_set.insert(5);
    collision_data.result.clear();

    mot_manager.collide(ego_cos_vec.at(8), &collision_data,
                        cdl::CollisionFunction);
    std::cout << "enable filter, but the trajectory id is not in the filter"
              << std::endl;
    std::cout << "pose vs tree contact num:"
              << collision_data.result.numContacts() << std::endl;
    if (collision_data.result.numContacts() > 0)
    {
        std::vector<cdl::Contact> collisions;
        collision_data.result.getContacts(collisions);
        std::stable_sort(collisions.begin(), collisions.end(), cdl::compare2ID);
        for (const auto& ct : collisions)
        {
            std::cout << "ct b1:" << ct.b2 << ", b2:" << ct.b1 << std::endl;
        }
    }

    // distance test
    collision_data.request.num_max_contacts = 10000;
    collision_data.request.enable_contact = true;
    collision_data.request.enable_filter = true;
    collision_data.request.distance_lower_bound = 5;
    collision_data.request.enable_distance_request = true;
    collision_data.request.filter_set.clear();
    // only detection collision with trajectories in the filter_set
    collision_data.request.filter_set.insert(mot_traj_id);
    collision_data.result.clear();
    std::size_t tag_obj = 6;
    mot_manager.distance(ego_cos_vec.at(tag_obj), &collision_data,
                         cdl::IsolationDistanceFunction);

    tagFootprint(ego_traj.at(tag_obj), footprint.get(), &viz, viz.black, 2);
    std::cout << "distance query, contact num:"
              << collision_data.result.numContacts() << " distance pair num:"
              << collision_data.result.numDistancePairs() << std::endl;

    for (const auto& dp : collision_data.result.getDistancePairs())
    {
        std::cout << "distance pair:" << dp.b1 << " ," << dp.b2 << std::endl;
        std::cout << "distance:" << dp.distance << std::endl;
        tagFootprint(mot_traj.at(dp.b1), footprint.get(), &viz, viz.black, 2);
    }

    for (std::size_t i = 0; i < ego_cos_vec.size(); ++i)
    {
        delete ego_cos_vec.at(i);
    }

    for (std::size_t i = 0; i < mot_cos_vec.size(); ++i)
    {
        delete mot_cos_vec.at(i);
    }

    viz.setDisplayData();
    viz.viewer->run();
}