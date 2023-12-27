/*
 * Copyright 2016-2022. collision
 * See LICENSE AGREEMENT file in the project root for full license information.
 */

#include "contact2d.h"

namespace cdl
{
namespace bp2d
{
Contact::Contact() : b1(NONE), b2(NONE) {}

Contact::Contact(int b1_, int b2_) : b1(b1_), b2(b2_) {}

Contact::Contact(int b1_, int b2_, const Vector2r &pos_,
                 const Vector2r &normal_, real depth_)
    : b1(b1_), b2(b2_), normal(normal_), pos(pos_), distance(depth_)
{
}

void Contact::set(const Contact &c)

{
    b1 = c.b1;
    b2 = c.b2;
    normal = c.normal;
    pos = c.pos;
    distance = c.distance;
}

void Contact::clear()

{
    b1 = 0;
    b2 = 0;
    normal = Vector2r::Zero();
    pos = cdl::Vector2r::Zero();
    distance = 0.0;
}

bool Contact::operator<(const Contact &other) const
{
    if (b1 == other.b1) return b2 < other.b2;
    return b1 < other.b1;
}
}  // namespace bp2d
}  // namespace cdl
