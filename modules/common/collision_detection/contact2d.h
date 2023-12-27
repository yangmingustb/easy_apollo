/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#pragma once
#include "collision_object2d.h"

namespace cdl
{
namespace bp2d
{
/**
 * \brief  Contact struct is used to hold contact information or distance
 * information by the collision detection algorithms.
 */
struct CDL_EXPORT Contact
{
    /** collision object id in manager 1 */
    int b1;

    /** collision object id in in manager 2 */
    int b2;

    /** contact normal, pointing from o1 to o2 */
    Vector2r normal;

    /** contact position, in world space */
    Vector2r pos;

    /** distance, can be positive, zero and negative **/
    real distance;

    /** invalid contact primitive information */
    static const int NONE = -1;

    Contact();
    Contact(int b1_, int b2_);
    Contact(int b1_, int b2_, const Vector2r &pos_, const Vector2r &normal_,
            real depth_);

    void set(const Contact &c);

    bool operator<(const Contact &other) const;

    void clear();
};  // class contact
}  // namespace bp2d
}  // namespace cdl
