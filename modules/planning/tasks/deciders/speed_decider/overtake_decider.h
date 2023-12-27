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
int estimate_min_lon_gap_for_overtake_decision(double* min_gap,
                                               const Obstacle& obstacle,
                                               const SpeedData* adc_speed_data);
}
}