/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "dynamic_tree.h"
#include <string.h>
#include <algorithm>
#include <iostream>
#include "morton2d.h"

namespace cdl
{
namespace bp2d
{
b2DynamicTree::b2DynamicTree()
{
    m_root = b2_nullNode;
    m_nodeCount = 0;
    m_path = 0;
    m_insertionCount = 0;
    m_nodes = nullptr;
    m_nodes_buf_size = 0;
}

b2DynamicTree::b2DynamicTree(b2TreeNode *pool, int32_t size)
{
    m_root = b2_nullNode;
    m_nodeCount = 0;
    m_path = 0;
    m_insertionCount = 0;
    m_nodes = pool;
    m_nodes_buf_size = size;
}

void b2DynamicTree::set_tree_node_pool(b2TreeNode *pool, int32_t size)
{
    m_nodes = pool;
    m_nodes_buf_size = size;
}

void b2DynamicTree::clear()
{
    m_root = 0;
    m_nodeCount = 0;
    m_path = 0;
    m_insertionCount = 0;
}

void *b2DynamicTree::GetUserData(int32 proxyId) const
{
    return m_nodes[proxyId].userData;
}

int32 b2DynamicTree::AllocateNode()
{
    /** Peel a node off the free list. */
    int32 nodeId = m_nodeCount;
    m_nodes[nodeId].parent = b2_nullNode;
    m_nodes[nodeId].child1 = b2_nullNode;
    m_nodes[nodeId].child2 = b2_nullNode;
    m_nodes[nodeId].height = 0;
    m_nodes[nodeId].userData = nullptr;
    ++m_nodeCount;
    return nodeId;
}

void b2DynamicTree::FreeNode(int32 nodeId)
{
    m_nodes[nodeId].height = -1;
    --m_nodeCount;
}

int32 b2DynamicTree::CreateProxy(const AABB &aabb, void *userData)
{
    int32 proxyId = AllocateNode();

    m_nodes[proxyId].aabb = aabb;
    m_nodes[proxyId].userData = userData;
    m_nodes[proxyId].height = 0;

    InsertLeaf(proxyId);

    return proxyId;
}

bool sortByMorton(const b2TreeNode &a, const b2TreeNode &b)
{
    return a.code < b.code;
}

int32 b2DynamicTree::CreateProxy(b2TreeNode *objects, const int32 &size,
                                 b2TreeNode *tmp_buffer)
{
    int32 i;
    m_insertionCount = size;
    AABB bound_bv;

    if (size > 0)
    {
        bound_bv = objects[0].aabb;
    }
    for (i = 1; i < size; ++i)
    {
        bound_bv.combine(objects[i].aabb);
    }

    morton_functoru32r coder(bound_bv);
    for (i = 0; i < size; ++i)
    {
        objects[i].code =
                coder(objects[i].aabb.center()); /**< generate morton code */
    }

    /** sort by morton code: 1. STL.sort, 2. radix sort(faster). The use way:
     * radixSort(objects, size);
     * std::sort(objects, objects + size, sortByMorton);
     */
    radixSort(objects, size, tmp_buffer);

    for (i = 0; i < size; ++i)
    {
        m_nodes[i].height = 0;
        m_nodes[i].child1 = b2_nullNode;
        m_nodes[i].child2 = b2_nullNode;
    }

    m_nodeCount = size;

    m_root = mortonRecurse_3(0, size);
    /** you can also call recurseRefit(m_root); */
    refit();

    return 0;
}

void b2DynamicTree::recurseRefit(const int32 node)
{
    if (!m_nodes[node].IsLeaf())
    {
        int32 child1 = m_nodes[node].child1;
        int32 child2 = m_nodes[node].child2;

        recurseRefit(child1);
        recurseRefit(child2);

        m_nodes[node].aabb.combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
    }
    else
        return;
}

void b2DynamicTree::refit()
{
    int32_t i;
    int32_t child1, child2;

    if (m_insertionCount == 1)
    {
        return;
    }
    for (i = m_nodeCount - 1; i >= m_root; i--)
    {
        child1 = m_nodes[i].child1;
        child2 = m_nodes[i].child2;
        m_nodes[i].aabb.combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
    }
}

int32 b2DynamicTree::mortonRecurse_2(const int32 lbeg, const int32 lend)
{
    int num_leaves = lend - lbeg;
    if (num_leaves > 1)
    {
        int32 child1 = mortonRecurse_2(lbeg, lbeg + num_leaves / 2);
        int32 child2 = mortonRecurse_2(lbeg + num_leaves / 2, lend);
        int32 id = AllocateNode();
        m_nodes[id].child1 = child1;
        m_nodes[id].child2 = child2;

        m_nodes[child1].parent = id;
        m_nodes[child2].parent = id;
        return id;
    }
    else
        return lbeg;
}

int32 b2DynamicTree::mortonRecurse_3(const int32 lbeg, const int32 lend)
{
    int num_leaves = lend - lbeg;

    if (num_leaves > 1)
    {
        int32 n_begin, n_end, lcenter;
        int32 parent_node_id;
        int32 child1_id, child2_id;

        int32 root_id;
        int32 queue_size = 0;
        root_id = AllocateNode();
        m_nodes[root_id].child1 = lbeg;
        m_nodes[root_id].child2 = lend;

        queue_size++;

        /** internal node initial flag */
        int32 internal_node_id = 0;
        while (queue_size > 0)
        {
            parent_node_id = m_insertionCount + internal_node_id;

            n_begin = m_nodes[parent_node_id].child1;
            n_end = m_nodes[parent_node_id].child2;
            num_leaves = n_end - n_begin;

            lcenter = n_begin + num_leaves / 2;

            queue_size--;
            if (lcenter - n_begin > 1)
            {
                queue_size++;
                child1_id = AllocateNode();
                m_nodes[child1_id].child1 = n_begin;
                m_nodes[child1_id].child2 = lcenter;
            }
            else
            {
                child1_id = n_begin;
            }

            if (n_end - lcenter > 1)
            {
                queue_size++;
                child2_id = AllocateNode();
                m_nodes[child2_id].child1 = lcenter;
                m_nodes[child2_id].child2 = n_end;
            }
            else
            {
                child2_id = lcenter;
            }

            m_nodes[parent_node_id].child1 = child1_id;
            m_nodes[parent_node_id].child2 = child2_id;
            m_nodes[child1_id].parent = parent_node_id;
            m_nodes[child2_id].parent = parent_node_id;
            internal_node_id++;
        }

        return root_id;
    }

    return lbeg;
};

void b2DynamicTree::radixSort(b2TreeNode *nodes, const int32 size,
                              b2TreeNode *tempVector)
{
    int32 bitsPerPass = 5;
    int32 nBits = 20;
    int32 nPasses = nBits / bitsPerPass;

    int32 pass;
    int32 lowBit;
    b2TreeNode *in, *out;
    int32 nBuckets;
    int32 bitMask;
    int32 i;
    int32 bucket;
    nBuckets = 1 << bitsPerPass;
    bitMask = (1 << bitsPerPass) - 1;

    int32 bucketCount[nBuckets];
    int32 outIndex[nBuckets];

    for (pass = 0; pass < nPasses; ++pass)
    {
        /** Perform one pass of radix sort, sorting _bitsPerPass_ bits */
        lowBit = pass * bitsPerPass;

        /** Set in and out vector pointers for radix sort pass */
        in = (pass & 1) ? tempVector : nodes;
        out = (pass & 1) ? nodes : tempVector;

        /** Count number of zero bits in array for current radix sort bit */
        for (i = 0; i < nBuckets; i++)
        {
            bucketCount[i] = 0;;
        }
        for (i = 0; i < size; ++i)
        {
            bucket = (in[i].code >> lowBit) & bitMask;
            ++bucketCount[bucket];
        }

        /** Compute starting index in output array for each bucket */
        for (i = 0; i < nBuckets; i++)
        {
            outIndex[i] = 0;
        }
        for (i = 1; i < nBuckets; ++i)
        {
            outIndex[i] = outIndex[i - 1] + bucketCount[i - 1];
        }

        /** Store sorted values in output array */
        for (i = 0; i < size; ++i)
        {
            bucket = (in[i].code >> lowBit) & bitMask;
            out[outIndex[bucket]++] = in[i];
        }
    }
    /** Copy final result from _tempVector_, if needed */
    if (nPasses & 1) nodes = tempVector;
}

void b2DynamicTree::updateParentAABB(const int32 id)
{
    int32 child1 = m_nodes[id].child1;
    int32 child2 = m_nodes[id].child2;
    m_nodes[id].aabb.combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
}

const b2TreeNode *b2DynamicTree::getRootNode() const
{
    return m_nodes + m_root;
}

void b2DynamicTree::DestroyProxy(int32 proxyId)
{
    RemoveLeaf(proxyId);
    FreeNode(proxyId);
}

void b2DynamicTree::maxBoundingBox(AABB *bv, b2TreeNode *objects,
                                   const int32 size) const
{
    cdl::Vector2r min_ = objects[0].aabb.min_;
    cdl::Vector2r max_ = objects[0].aabb.max_;
    for (int32 i = 1; i < size; ++i)
    {
        if (min_(0) > objects[i].aabb.min_(0))
        {
            min_(0) = objects[i].aabb.min_(0);
        }
        if (min_(1) > objects[i].aabb.min_(1))
        {
            min_(1) = objects[i].aabb.min_(1);
        }
        if (max_(0) < objects[i].aabb.max_(0))
        {
            max_(0) = objects[i].aabb.max_(0);
        }
        if (max_(1) < objects[i].aabb.max_(1))
        {
            max_(1) = objects[i].aabb.max_(1);
        }
    }
    bv->min_ = min_;
    bv->max_ = max_;
};

void b2DynamicTree::InsertLeaf(int32 leaf)
{
    ++m_insertionCount;

    if (m_root == b2_nullNode)
    {
        m_root = leaf;
        m_nodes[m_root].parent = b2_nullNode;
        return;
    }

    /** Find the best sibling for this node */
    AABB leafAABB = m_nodes[leaf].aabb;
    int32 index = m_root;
    while (m_nodes[index].IsLeaf() == false)
    {
        int32 child1 = m_nodes[index].child1;
        int32 child2 = m_nodes[index].child2;

        real area = m_nodes[index].aabb.perimeter();

        AABB combinedAABB;
        combinedAABB = m_nodes[index].aabb + leafAABB;
        real combinedArea = combinedAABB.perimeter();

        /** Cost of creating a new parent for this node and the new leaf */
        real cost = 2.0 * combinedArea;

        /** Minimum cost of pushing the leaf further down the tree */
        real inheritanceCost = 2.0 * (combinedArea - area);

        /** Cost of descending into child1 */
        real cost1;
        if (m_nodes[child1].IsLeaf())
        {
            AABB aabb;
            aabb = leafAABB + m_nodes[child1].aabb;
            cost1 = aabb.perimeter() + inheritanceCost;
        }
        else
        {
            AABB aabb;
            aabb = leafAABB + m_nodes[child1].aabb;
            real oldArea = m_nodes[child1].aabb.perimeter();
            real newArea = aabb.perimeter();
            cost1 = newArea - oldArea + inheritanceCost;
        }

        /** Cost of descending into child2 */
        real cost2;
        if (m_nodes[child2].IsLeaf())
        {
            AABB aabb;
            aabb = leafAABB + m_nodes[child2].aabb;
            cost2 = aabb.perimeter() + inheritanceCost;
        }
        else
        {
            AABB aabb;
            aabb = leafAABB + m_nodes[child2].aabb;
            real oldArea = m_nodes[child2].aabb.perimeter();
            real newArea = aabb.perimeter();
            cost2 = newArea - oldArea + inheritanceCost;
        }

        /** Descend according to the minimum cost. */
        if (cost < cost1 && cost < cost2)
        {
            break;
        }

        /** Descend */
        if (cost1 < cost2)
        {
            index = child1;
        }
        else
        {
            index = child2;
        }
    }

    int32 sibling = index;

    /** Create a new parent. */
    int32 oldParent = m_nodes[sibling].parent;
    int32 newParent = AllocateNode();
    m_nodes[newParent].parent = oldParent;
    m_nodes[newParent].userData = nullptr;
    m_nodes[newParent].aabb = leafAABB + m_nodes[sibling].aabb;
    m_nodes[newParent].height = m_nodes[sibling].height + 1;

    if (oldParent != b2_nullNode)
    {
        /** The sibling was not the root. */
        if (m_nodes[oldParent].child1 == sibling)
        {
            m_nodes[oldParent].child1 = newParent;
        }
        else
        {
            m_nodes[oldParent].child2 = newParent;
        }

        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = leaf;
        m_nodes[sibling].parent = newParent;
        m_nodes[leaf].parent = newParent;
    }
    else
    {
        /** The sibling was the root. */
        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = leaf;
        m_nodes[sibling].parent = newParent;
        m_nodes[leaf].parent = newParent;
        m_root = newParent;
    }

    /** Walk back up the tree fixing heights and AABBs */
    index = m_nodes[leaf].parent;
    while (index != b2_nullNode)
    {
        index = Balance(index);

        int32 child1 = m_nodes[index].child1;
        int32 child2 = m_nodes[index].child2;

        m_nodes[index].height =
                1 + std::max(m_nodes[child1].height, m_nodes[child2].height);
        m_nodes[index].aabb = m_nodes[child1].aabb + m_nodes[child2].aabb;

        index = m_nodes[index].parent;
    }
}

void b2DynamicTree::RemoveLeaf(int32 leaf)
{
    if (leaf == m_root)
    {
        m_root = b2_nullNode;
        return;
    }

    int32 parent = m_nodes[leaf].parent;
    int32 grandParent = m_nodes[parent].parent;
    int32 sibling;
    if (m_nodes[parent].child1 == leaf)
    {
        sibling = m_nodes[parent].child2;
    }
    else
    {
        sibling = m_nodes[parent].child1;
    }

    if (grandParent != b2_nullNode)
    {
        /** Destroy parent and connect sibling to grandParent. */
        if (m_nodes[grandParent].child1 == parent)
        {
            m_nodes[grandParent].child1 = sibling;
        }
        else
        {
            m_nodes[grandParent].child2 = sibling;
        }
        m_nodes[sibling].parent = grandParent;
        FreeNode(parent);

        /** Adjust ancestor bounds. */
        int32 index = grandParent;
        while (index != b2_nullNode)
        {
            index = Balance(index);

            int32 child1 = m_nodes[index].child1;
            int32 child2 = m_nodes[index].child2;

            m_nodes[index].aabb = m_nodes[child1].aabb + m_nodes[child2].aabb;
            m_nodes[index].height = 1 + std::max(m_nodes[child1].height,
                                                 m_nodes[child2].height);

            index = m_nodes[index].parent;
        }
    }
    else
    {
        m_root = sibling;
        m_nodes[sibling].parent = b2_nullNode;
        FreeNode(parent);
    }
}

/** Perform a left or right rotation if node A is imbalanced. Returns the new
 * root index.
 */
int32 b2DynamicTree::Balance(int32 iA)
{
    b2TreeNode *A = m_nodes + iA;
    if (A->IsLeaf() || A->height < 2)
    {
        return iA;
    }

    int32 iB = A->child1;
    int32 iC = A->child2;

    b2TreeNode *B = m_nodes + iB;
    b2TreeNode *C = m_nodes + iC;

    int32 balance = C->height - B->height;

    /** Rotate C up */
    if (balance > 1)
    {
        int32 iF = C->child1;
        int32 iG = C->child2;
        b2TreeNode *F = m_nodes + iF;
        b2TreeNode *G = m_nodes + iG;

        /** Swap A and C */
        C->child1 = iA;
        C->parent = A->parent;
        A->parent = iC;

        /** A's old parent should point to C */
        if (C->parent != b2_nullNode)
        {
            if (m_nodes[C->parent].child1 == iA)
            {
                m_nodes[C->parent].child1 = iC;
            }
            else
            {
                m_nodes[C->parent].child2 = iC;
            }
        }
        else
        {
            m_root = iC;
        }

        /** Rotate */
        if (F->height > G->height)
        {
            C->child2 = iF;
            A->child2 = iG;
            G->parent = iA;
            A->aabb = B->aabb + G->aabb;
            C->aabb = A->aabb + F->aabb;

            A->height = 1 + std::max(B->height, G->height);
            C->height = 1 + std::max(A->height, F->height);
        }
        else
        {
            C->child2 = iG;
            A->child2 = iF;
            F->parent = iA;
            A->aabb = B->aabb + F->aabb;
            C->aabb = A->aabb + G->aabb;

            A->height = 1 + std::max(B->height, F->height);
            C->height = 1 + std::max(A->height, G->height);
        }

        return iC;
    }

    /** Rotate B up */
    if (balance < -1)
    {
        int32 iD = B->child1;
        int32 iE = B->child2;
        b2TreeNode *D = m_nodes + iD;
        b2TreeNode *E = m_nodes + iE;

        /** Swap A and B */
        B->child1 = iA;
        B->parent = A->parent;
        A->parent = iB;

        /** A's old parent should point to B */
        if (B->parent != b2_nullNode)
        {
            if (m_nodes[B->parent].child1 == iA)
            {
                m_nodes[B->parent].child1 = iB;
            }
            else
            {
                m_nodes[B->parent].child2 = iB;
            }
        }
        else
        {
            m_root = iB;
        }

        /** Rotate */
        if (D->height > E->height)
        {
            B->child2 = iD;
            A->child1 = iE;
            E->parent = iA;
            A->aabb = C->aabb + E->aabb;
            B->aabb = A->aabb + D->aabb;

            A->height = 1 + std::max(C->height, E->height);
            B->height = 1 + std::max(A->height, D->height);
        }
        else
        {
            B->child2 = iE;
            A->child1 = iD;
            D->parent = iA;
            A->aabb = C->aabb + D->aabb;
            B->aabb = A->aabb + E->aabb;

            A->height = 1 + std::max(C->height, D->height);
            B->height = 1 + std::max(A->height, E->height);
        }

        return iB;
    }

    return iA;
}

int32 b2DynamicTree::GetHeight() const
{
    if (m_root == b2_nullNode)
    {
        return 0;
    }

    return ComputeHeight();
}

real b2DynamicTree::GetAreaRatio() const
{
    if (m_root == b2_nullNode)
    {
        return 0.0;
    }

    const b2TreeNode *root = m_nodes + m_root;
    real rootArea = root->aabb.perimeter();

    real totalArea = 0.0;
    for (uint32 i = 0; i < cdl::MAX_NODE_POOL_SIZE; ++i)
    {
        const b2TreeNode *node = m_nodes + i;
        if (node->height < 0)
        {
            continue;
        }

        totalArea += node->aabb.perimeter();
    }

    return totalArea / rootArea;
}

int32 b2DynamicTree::ComputeHeight(int32 nodeId) const
{
    const b2TreeNode *node = m_nodes + nodeId;

    if (node->IsLeaf())
    {
        return 0;
    }

    int32 height1 = ComputeHeight(node->child1);
    int32 height2 = ComputeHeight(node->child2);
    return 1 + std::max(height1, height2);
}

int32 b2DynamicTree::ComputeHeight() const
{
    int32 height = ComputeHeight(m_root);
    return height;
}

void b2DynamicTree::ValidateStructure(int32 index) const
{
    if (index == b2_nullNode)
    {
        return;
    }

    const b2TreeNode *node = m_nodes + index;

    int32 child1 = node->child1;
    int32 child2 = node->child2;

    if (node->IsLeaf())
    {
        return;
    }

    ValidateStructure(child1);
    ValidateStructure(child2);
}

void b2DynamicTree::ValidateMetrics(int32 index) const
{
    if (index == b2_nullNode)
    {
        return;
    }

    const b2TreeNode *node = m_nodes + index;

    int32 child1 = node->child1;
    int32 child2 = node->child2;

    if (node->IsLeaf())
    {
        return;
    }

    ValidateMetrics(child1);
    ValidateMetrics(child2);
}

int32 b2DynamicTree::GetMaxBalance() const
{
    int32 maxBalance = 0;
    for (uint32 i = 0; i < cdl::MAX_NODE_POOL_SIZE; ++i)
    {
        const b2TreeNode *node = m_nodes + i;
        if (node->height <= 1)
        {
            continue;
        }

        int32 child1 = node->child1;
        int32 child2 = node->child2;
        int32 balance = abs(m_nodes[child2].height - m_nodes[child1].height);
        maxBalance = std::max(maxBalance, balance);
    }

    return maxBalance;
}

#if 0
void b2DynamicTree::RebuildBottomUp()
{
    int32 *nodes = (int32 *)malloc(m_nodeCount * sizeof(int32));
    printf("********allocate  *********\n");
    int32 count = 0;

    /** Build array of leaves. Free the rest. */
    for (uint32 i = 0; i < cdl::MAX_NODE_POOL_SIZE; ++i)
    {
        if (m_nodes[i].height < 0)
        {
            /** free node in pool */
            continue;
        }

        if (m_nodes[i].IsLeaf())
        {
            m_nodes[i].parent = b2_nullNode;
            nodes[count] = i;
            ++count;
        }
        else
        {
            FreeNode(i);
        }
    }

    while (count > 1)
    {
        real minCost = real_max;
        int32 iMin = -1, jMin = -1;
        for (int32 i = 0; i < count; ++i)
        {
            AABB aabbi = m_nodes[nodes[i]].aabb;

            for (int32 j = i + 1; j < count; ++j)
            {
                AABB aabbj = m_nodes[nodes[j]].aabb;
                AABB b = aabbi + aabbj;
                real cost = b.perimeter();
                if (cost < minCost)
                {
                    iMin = i;
                    jMin = j;
                    minCost = cost;
                }
            }
        }

        int32 index1 = nodes[iMin];
        int32 index2 = nodes[jMin];
        b2TreeNode *child1 = m_nodes + index1;
        b2TreeNode *child2 = m_nodes + index2;

        int32 parentIndex = AllocateNode();
        b2TreeNode *parent = m_nodes + parentIndex;
        parent->child1 = index1;
        parent->child2 = index2;
        parent->height = 1 + std::max(child1->height, child2->height);
        parent->aabb = child1->aabb + child2->aabb;
        parent->parent = b2_nullNode;

        child1->parent = parentIndex;
        child2->parent = parentIndex;

        nodes[jMin] = nodes[count - 1];
        nodes[iMin] = parentIndex;
        --count;
    }

    m_root = nodes[0];
}
#endif

void b2DynamicTree::ShiftOrigin(const Vector2r &newOrigin)
{
    /** Build array of leaves. Free the rest. */
    for (uint32 i = 0; i < cdl::MAX_NODE_POOL_SIZE; ++i)
    {
        m_nodes[i].aabb.min_ -= newOrigin;
        m_nodes[i].aabb.max_ -= newOrigin;
    }
}

bool b2DynamicTree::CollisionRecurse(const int32 root_id1, const int32 root_id2,
                                     b2DynamicTree *tree)
{
    const b2TreeNode *root1 = m_nodes + root_id1;
    const b2TreeNode *root2 = tree->m_nodes + root_id2;
    if (root1->IsLeaf() && root2->IsLeaf())
    {
        if (!(root1->aabb.overlap(root2->aabb)))
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    if (!(root1->aabb.overlap(root2->aabb)))
    {
        return false;
    }

    if (root2->IsLeaf() ||
        (!root1->IsLeaf() && (root1->aabb.size() > root2->aabb.size())))
    {
        if (CollisionRecurse(root1->child1, root_id2, tree)) return true;
        if (CollisionRecurse(root1->child2, root_id2, tree)) return true;
    }
    else
    {
        if (CollisionRecurse(root_id1, root2->child1, tree)) return true;
        if (CollisionRecurse(root_id1, root2->child2, tree)) return true;
    }
    return false;
}

bool b2DynamicTree::CollisionRecurse(const int32 root_id1, const int32 root_id2,
                                     b2DynamicTree *tree, CollisionData *cdata)
{
    const b2TreeNode *root1 = getNode(root_id1);
    const b2TreeNode *root2 = tree->getNode(root_id2);

    if (root1->IsLeaf() && root2->IsLeaf())
    {
        if (!(root1->aabb.overlap(root2->aabb)))
        {
            return false;
        }
        else
        {
            const auto &request = cdata->request;
            auto &result = cdata->result;
            if (request.num_max_contacts <= result.numContacts())
            {
                cdata->done = true;
            }
            else
            {
                Vector2r pt1, pt2, normal;
                real dist = root1->aabb.distance(root2->aabb, &pt1, &pt2);
                normal = pt2 - pt1;
                Contact ct(Contact::NONE, Contact::NONE, pt1, normal, dist);
                result.addContact(ct);
                if (result.isCollision() &&
                    request.num_max_contacts >= result.numContacts())
                {
                    cdata->done = true;
                }
                return false;
            }
        }
    }
    if (!(root1->aabb.overlap(root2->aabb)))
    {
        return false;
    }

    if (root2->IsLeaf() ||
        (!root1->IsLeaf() && (root1->aabb.size() > root2->aabb.size())))
    {
        if (CollisionRecurse(root1->child1, root_id2, tree, cdata)) return true;
        if (CollisionRecurse(root1->child2, root_id2, tree, cdata)) return true;
    }
    else
    {
        if (CollisionRecurse(root_id1, root2->child1, tree, cdata)) return true;
        if (CollisionRecurse(root_id1, root2->child2, tree, cdata)) return true;
    }
    return false;
}

bool b2DynamicTree::CollisionRecurse(const int32 root_id1, const int32 root_id2,
                                     b2DynamicTree *tree, CollisionData *cdata,
                                     b2_CollisionCallBack callback) const
{
    const b2TreeNode *root1 = getNode(root_id1);
    const b2TreeNode *root2 = tree->getNode(root_id2);

    if (root1->IsLeaf() && root2->IsLeaf())
    {
        if (!(root1->aabb.overlap(root2->aabb)))
        {
            return false;
        }
        else
        {
            auto &result = cdata->result;
            result.aabb_collision_pair++;
            return callback(static_cast<CollisionObject *>(root1->userData),
                            static_cast<CollisionObject *>(root2->userData),
                            cdata);
        }
    }
    if (!(root1->aabb.overlap(root2->aabb)))
    {
        return false;
    }

    if (root2->IsLeaf() ||
        (!root1->IsLeaf() && (root1->aabb.size() > root2->aabb.size())))
    {
        if (CollisionRecurse(root1->child1, root_id2, tree, cdata, callback))
            return true;
        if (CollisionRecurse(root1->child2, root_id2, tree, cdata, callback))
            return true;
    }
    else
    {
        if (CollisionRecurse(root_id1, root2->child1, tree, cdata, callback))
            return true;
        if (CollisionRecurse(root_id1, root2->child2, tree, cdata, callback))
            return true;
    }
    return false;
}

bool b2DynamicTree::DistanceRecurse(const int32 root_id1, const int32 root_id2,
                                    b2DynamicTree *tree, CollisionData *cdata,
                                    b2_DistanceCallBack callback,
                                    real &min_dist) const
{
    const b2TreeNode *root1 = getNode(root_id1);
    const b2TreeNode *root2 = tree->getNode(root_id2);

    if (root1->IsLeaf() && root2->IsLeaf())
    {
        if (root1->aabb.distance(root2->aabb) > min_dist)
        {
            return false;
        }

        else
        {
            CollisionObject *root1_obj =
                    static_cast<CollisionObject *>(root1->userData);
            CollisionObject *root2_obj =
                    static_cast<CollisionObject *>(root2->userData);

            return callback(root1_obj, root2_obj, cdata, min_dist);
        }
    }
    if (root2->IsLeaf() ||
        (!root1->IsLeaf() && (root1->aabb.size() > root2->aabb.size())))
    {
        const b2TreeNode *root_child1 = getNode(root1->child1);
        const b2TreeNode *root_child2 = getNode(root1->child2);
        real d1 = root2->aabb.distance(root_child1->aabb);
        real d2 = root2->aabb.distance(root_child2->aabb);

        if (d2 < d1)
        {
            if (d2 < min_dist)
            {
                if (DistanceRecurse(root1->child2, root_id2, tree, cdata,
                                    callback, min_dist))
                    return true;
            }

            if (d1 < min_dist)
            {
                if (DistanceRecurse(root1->child1, root_id2, tree, cdata,
                                    callback, min_dist))
                    return true;
            }
        }
        else
        {
            if (d1 < min_dist)
            {
                if (DistanceRecurse(root1->child1, root_id2, tree, cdata,
                                    callback, min_dist))
                    return true;
            }

            if (d2 < min_dist)
            {
                if (DistanceRecurse(root1->child2, root_id2, tree, cdata,
                                    callback, min_dist))
                    return true;
            }
        }
    }
    else
    {
        const b2TreeNode *root2_child1 = tree->getNode(root2->child1);
        const b2TreeNode *root2_child2 = tree->getNode(root2->child2);
        real d1 = root1->aabb.distance(root2_child1->aabb);
        real d2 = root1->aabb.distance(root2_child2->aabb);

        if (d2 < d1)
        {
            if (d2 < min_dist)
            {
                if (DistanceRecurse(root_id1, root2->child2, tree, cdata,
                                    callback, min_dist))
                    return true;
            }

            if (d1 < min_dist)
            {
                if (DistanceRecurse(root_id1, root2->child1, tree, cdata,
                                    callback, min_dist))
                    return true;
            }
        }
        else
        {
            if (d1 < min_dist)
            {
                if (DistanceRecurse(root_id1, root2->child1, tree, cdata,
                                    callback, min_dist))
                    return true;
            }

            if (d2 < min_dist)
            {
                if (DistanceRecurse(root_id1, root2->child2, tree, cdata,
                                    callback, min_dist))
                    return true;
            }
        }
    }

    return false;
}

void b2DynamicTree::TreeVsTree(b2DynamicTree *tree)
{
    if (m_nodeCount == 0 || tree->getToalNodeNumber() == 0)
    {
        return;
    }
    CollisionRecurse(m_root, tree->m_root, tree);
}

void b2DynamicTree::TreeVsTree(b2DynamicTree *tree, CollisionData *cdata)
{
    if (m_nodeCount == 0 || tree->getToalNodeNumber() == 0)
    {
        return;
    }
    CollisionRecurse(m_root, tree->m_root, tree, cdata);
}

void b2DynamicTree::TreeVsTree(b2DynamicTree *tree, CollisionData *cdata,
                               b2_CollisionCallBack callback) const
{
    if (m_nodeCount == 0 || tree->getToalNodeNumber() == 0)
    {
        return;
    }
    CollisionRecurse(m_root, tree->m_root, tree, cdata, callback);
}

void b2DynamicTree::TreeVsTree(b2DynamicTree *tree, CollisionData *cdata,
                               b2_DistanceCallBack callback) const
{
    if (m_nodeCount == 0 || tree->getToalNodeNumber() == 0)
    {
        return;
    }
    real min_dist = std::numeric_limits<real>::max();
    DistanceRecurse(m_root, tree->m_root, tree, cdata, callback, min_dist);
}

}  // namespace bp2d
}  // namespace cdl
