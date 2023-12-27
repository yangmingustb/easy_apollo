#pragma once


#include "modules/common/status/system_state.h"
#include "opencv_viz.h"


#include <iostream>
#include <memory>

namespace apollo
{

struct cv_direction_key
{
    bool left;
    bool right;
    bool up;
    bool low;
};

void cv_key_direction_init(cv_direction_key* key);

bool has_cv_direction_key(cv_direction_key* key);
}