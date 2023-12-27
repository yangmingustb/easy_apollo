/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#ifndef DYNAMIC_TREE_H
#define DYNAMIC_TREE_H

#include <iostream>
#include "aabb2d.h"
#include "collision_data2d.h"
#include "collision_object2d.h"
#include "constants.h"
#include "node_base2d.h"
#include "types.h"

namespace cdl
{
namespace bp2d
{
enum struct TreeBuildAlgo2
{
    MIDDLE_SPLIT,
    EQUAL_COUNT_SPLIT,
    SAH,
    LBVH_RECURSE,
    LBVH_ITERATION,
    ALGO_NUM
};

enum struct TreeSortAlgo
{
    STL_SORT,
    RADIX_SORT,
    SORT_NUM
};

/**
 * \brief Callback for collision between two objects. Return value is whether
 * can stop now.
 */
using b2_CollisionCallBack = bool (*)(CollisionObject *o1, CollisionObject *o2,
                                      CollisionData *cdata);

/**
 * \brief Callback for distance between two objects, Return value is whether
 * can stop now, also return the minimum distance till now.
 */
using b2_DistanceCallBack = bool (*)(CollisionObject *o1, CollisionObject *o2,
                                     CollisionData *cdata, real &min_distance);

class b2DynamicTree
{
public:
    /** Constructing a tree using nodes pool. */
    b2DynamicTree();
    b2DynamicTree(b2TreeNode *pool, int32_t size);

    void set_tree_node_pool(b2TreeNode *pool, int32_t size);
    void clear();

    /** Create a proxy using aabb */
    int32 CreateProxy(const AABB &aabb, void *userData);

    /** Create a tree using TreeNodes */
    int32 CreateProxy(b2TreeNode *objects, const int32 &size,
                      b2TreeNode *tmp_buffer);

    /** Destroy a proxy. This asserts if the id is invalid. */
    void DestroyProxy(int32 proxyId);

    /** Get proxy user data. return the proxy user data or 0 if the id is
     * invalid. */
    void *GetUserData(int32 proxyId) const;

    bool WasMoved(int32 proxyId) const;

    void ClearMoved(int32 proxyId);

    /** Get the fat AABB for a proxy. */
    const AABB &GetFatAABB(int32 proxyId) const;

    /** query aabb in the tree */
    void Query(const AABB &aabb) const;

    /** tree vs tree query */
    void TreeVsTree(b2DynamicTree *tree);

    void TreeVsTree(b2DynamicTree *tree, CollisionData *cdata);

    /** perform collision test with objects belonging to another manager */
    void TreeVsTree(b2DynamicTree *tree, CollisionData *cdata,
                    b2_CollisionCallBack callback) const;

    /** perform collision test with objects belonging to another manager */
    void TreeVsTree(b2DynamicTree *tree, CollisionData *cdata,
                    b2_DistanceCallBack callback) const;

    bool CollisionRecurse(const int32 root, const int32 another_root,
                          b2DynamicTree *tree);

    bool CollisionRecurse(const int32 root_id1, const int32 root_id2,
                          b2DynamicTree *tree, CollisionData *cdata);

    bool CollisionRecurse(const int32 root_id1, const int32 root_id2,
                          b2DynamicTree *tree, CollisionData *cdata,
                          b2_CollisionCallBack callback) const;

    bool DistanceRecurse(const int32 root_id1, const int32 root_id2,
                         b2DynamicTree *tree, CollisionData *cdata,
                         b2_DistanceCallBack callback, real &min_dist) const;

    /** Compute the height of the binary tree in O(N) time. Should not be called
     * often. */
    int32 GetHeight() const;

    /** Get the maximum balance of an node in the tree. The balance is the
    difference in height of the two children of a node. */
    int32 GetMaxBalance() const;

    /** Get the ratio of the sum of the node areas to the root area. */
    real GetAreaRatio() const;

    /** Build an optimal tree. Very expensive. For testing. */
    void RebuildBottomUp();

    /** Shift the world origin. Useful for large worlds. The shift formula is:
    position -= newOrigin newOrigin the new origin with respect to the old
    origin */
    void ShiftOrigin(const Vector2r &newOrigin);

    int32 mortonRecurse_2(const int32 lbeg, const int32 lend);

    /** radix sorting for leaf nodes */
    void radixSort(b2TreeNode *nodes, const int32 size, b2TreeNode *tmp);

    /** morton code building using iteration way */
    int32 mortonRecurse_3(const int32 lbeg, const int32 lend);

    void recurseRefit(const int32 node);

    void refit();

    void updateParentAABB(const int32 id);

    /** internal node number */
    int32 getInternalNodeNumber() const
    {
        return m_nodeCount - m_insertionCount;
    }

    /** leaf node number */
    int32 getInsertNodeNumber() const { return m_insertionCount; }

    /** toal node number: leaf node + internal node */
    int32 getToalNodeNumber() const { return m_nodeCount; }

    int32 getRootID() const { return m_root; }

    const b2TreeNode *getRootNode() const;

    const b2TreeNode *getNode(const int32 id) const { return m_nodes + id; }

private:
    int32 AllocateNode();

    void FreeNode(int32 node);

    void InsertLeaf(int32 node);

    void RemoveLeaf(int32 node);

    int32 Balance(int32 index);

    /** calculate the height of the root node */
    int32 ComputeHeight() const;

    /** calculate the height of one node */
    int32 ComputeHeight(int32 nodeId) const;

    void ValidateStructure(int32 index) const;

    void ValidateMetrics(int32 index) const;

    /** generate bouding box using leaf nodes */
    void maxBoundingBox(AABB *bv, b2TreeNode *objects, const int32 size) const;

    /** root node id in nodes pool */
    int32 m_root;

    /** b2TreeNode m_nodes[cdl::MAX_NODE_POOL_SIZE] */
    b2TreeNode *m_nodes;
    int32 m_nodes_buf_size;

    /** Total node number：leaf node number+internal node number */
    int32 m_nodeCount;

    /** This is used to incrementally traverse the tree for re-balancing. */
    uint32 m_path;
    /** leaf node number */
    int32 m_insertionCount;
};

}  // namespace bp2d
}  // namespace cdl
#endif
