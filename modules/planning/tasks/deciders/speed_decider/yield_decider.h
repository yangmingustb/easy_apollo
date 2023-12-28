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

int estimate_proper_yield_poi(ObjectYield* yield_decision,
                                  const Obstacle& obstacle,
                                  const double adc_speed, const double adc_acc,
                                  const double path_length,
                                  const SpeedData* hard_brake_speed_data,
                                  const SpeedData* soft_brake_speed_data,
                                  const SLBoundary& adc_sl_boundary);

int estimate_yield_poi_by_two_phase_brake_curve(
        ObjectYield* yield_decision, const Obstacle& obstacle,
        const double adc_speed, const double adc_acc, const double path_length,
        const SpeedData* hard_brake_speed_data,
        const SpeedData* soft_brake_speed_data,
        const double emergency_brake_dist);
}
} // namespace apollo
