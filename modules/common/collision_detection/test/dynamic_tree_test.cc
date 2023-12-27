#include <gtest/gtest.h>
#include <iostream>
#include "dynamic_tree.h"
#include "morton2d.h"
#include "cdl_math.h"
#include "types.h"
#include "rng.h"
#include "modules/common/math/polygon_base.h"

using namespace cdl;
using namespace cdl::bp2d;
using namespace apollo;

#define DYNAMIC_TREE_SIZE_TEST (100)

TEST(test_b2DynamicTree, test_radixSort)
{
    int32_t i;
    b2TreeNode node[DYNAMIC_TREE_SIZE_TEST];
    b2TreeNode temp[DYNAMIC_TREE_SIZE_TEST];
    Vector2r  a(0.0, 0.0), b(1.0, 1.0);
    AABB bound_bv(a, b);
    b2DynamicTree obs_tree;

    double lowBound = -1000.0;
    double upBound = 1000.0;

    for (i = 0; i < DYNAMIC_TREE_SIZE_TEST; i++)
    {
        cdl::RNG rng;
        Vector2r  c(rng.uniformReal(lowBound, upBound), rng.uniformReal(lowBound, upBound));
        Vector2r  d(rng.uniformReal(lowBound, upBound), rng.uniformReal(lowBound, upBound));
        AABB aabb(c, d);
        node[i].aabb = aabb;
    }

    morton_functoru32r coder(bound_bv);
    for (i = 0; i < DYNAMIC_TREE_SIZE_TEST; ++i)
    {
        node[i].code =
                coder(node[i].aabb.center()); /**< generate morton code */
    }

    obs_tree.radixSort(node, DYNAMIC_TREE_SIZE_TEST, temp);

    /* after sort, the code are in none-decreasing order */
    for (i = 0; i < DYNAMIC_TREE_SIZE_TEST - 1; ++i)
    {
        EXPECT_LE(node[i].code, node[i+1].code);
    }
}

