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

int estimate_proper_following_gap(ObjectFollow* follow_decision,
                                  const Obstacle& obstacle,
                                  const double adc_speed, const double adc_acc,
                                  const double path_length,
                                  const SpeedData* hard_brake_speed_data,
                                  const SpeedData* soft_brake_speed_data,
                                  const SLBoundary& adc_sl_boundary,
                                  const double speed_limit);

double estimate_min_lon_dist_gap_for_follow_decision(
        const Obstacle& obstacle, const SpeedData& adc_speed_data);

int estimate_min_lon_dist_gap_for_follow_decision(
        follow_interaction_info* interaction_info, const Obstacle& obstacle,
        const SpeedData* adc_speed_data);

int estimate_following_acc_mode(ObjectFollow* follow_decision,
                                const Obstacle& obstacle,
                                const double adc_speed, const double adc_acc,
                                const double path_length,
                                const SpeedData* hard_brake_speed_data,
                                const SpeedData* soft_brake_speed_data,
                                const double speed_limit);
}
} // namespace apollo
