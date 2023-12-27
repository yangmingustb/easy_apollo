/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#include "box2d.h"
#include "collision_object2d.h"
#include "constants.h"
#include "convex2d.h"
#include "dynamic_tree.h"
#include "node_base2d.h"
#include "types.h"

namespace cdl
{
struct UserData
{
    /** id of collision object in the collision object array */
    int32 id;
    /** id of the trajecotry array in predicted trajectory set */
    int32 traj_id;
    /** id of the pose along a certain trajectory */
    int32 pose_id;

    void setUserData(const UserData &ud)
    {
        id = ud.id;
        traj_id = ud.traj_id;
        pose_id = ud.pose_id;
    }
};

class BVH_Data
{
public:
    cdl::bp2d::Box veh_shape_array[MAX_VEH_TRAJ_SIZE];
    cdl::bp2d::Convex obs_shape_array[MAX_LEAF_NODE_NUM];

    cdl::bp2d::CollisionObject veh_object_array[MAX_VEH_TRAJ_SIZE];
    cdl::bp2d::CollisionObject obs_object_array[MAX_LEAF_NODE_NUM];

    cdl::UserData veh_ud_array[MAX_VEH_TRAJ_SIZE];
    cdl::UserData obs_ud_array[MAX_LEAF_NODE_NUM];

    int32 veh_num, obs_num;

    cdl::bp2d::b2DynamicTree veh_tree, obs_tree;
    cdl::bp2d::CollisionData collision_data;

    cdl::bp2d::b2TreeNode veh_nodes[MAX_VEH_TRAJ_SIZE];
    cdl::bp2d::b2TreeNode obs_nodes[MAX_LEAF_NODE_NUM];

    void clear()
    {
        veh_num = 0;
        obs_num = 0;
    }
};

}  // namespace cdl