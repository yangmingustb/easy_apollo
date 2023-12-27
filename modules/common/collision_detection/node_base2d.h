/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once

#include "aabb2d.h"
#include "types.h"

namespace cdl
{
namespace bp2d
{
#define b2_nullNode (-1)

struct b2TreeNode
{
    bool IsLeaf() const { return child1 == b2_nullNode; }

    AABB aabb;

    void *userData;

    union
    {
        int32 parent;
        int32 next;
    };

    int32 child1;
    int32 child2;

    int32 height;
    int32 code;
};

}  // namespace bp2d
}  // namespace cdl
