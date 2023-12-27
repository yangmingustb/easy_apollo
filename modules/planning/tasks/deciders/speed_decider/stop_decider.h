#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "modules/planning/common/frame.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/tasks/task.h"

namespace apollo
{

namespace planning
{

int estimate_stop_dist_for_reverse_driving_obs(double* path_efence_s,
                                               const Obstacle& obstacle,
                                               const double adc_speed);
}  // namespace planning

} // namespace apollo
