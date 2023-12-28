
#include "follow_decider.h"
#include "yield_decider.h"
#include "modules/planning/common/speed_profile_generator.h"


namespace apollo
{
namespace planning
{

#define debug_yield_decider (0)

int estimate_yield_poi_by_two_phase_brake_curve(ObjectYield* yield_decision,
                             const Obstacle& obstacle, const double adc_speed,
                             const double adc_acc, const double path_length,
                             const SpeedData* hard_brake_speed_data,
                             const SpeedData* soft_brake_speed_data,
                             const double emergency_brake_dist)
{

    // init
    yield_decision->set_distance_s(-FLAGS_yield_distance);

#if debug_yield_decider

    const STBoundary& st_bound = obstacle.path_st_boundary();

    AINFO << "obs id: " << obstacle.Id();

    AINFO << "start collision, t,s: " << st_bound.min_t() << " , "
          << st_bound.min_s();

    AINFO << yield_decision->DebugString();
#endif

    return 0;
}

// TODO(Jinyun): add more variables to yield gap calculation
int estimate_proper_yield_poi(ObjectYield* yield_decision,
                              const Obstacle& obstacle, const double adc_speed,
                              const double adc_acc, const double path_length,
                              const SpeedData* hard_brake_speed_data,
                              const SpeedData* soft_brake_speed_data,
                              const SLBoundary& adc_sl_boundary)
{
    if (hard_brake_speed_data == nullptr || soft_brake_speed_data == nullptr)
    {
        AERROR << "No valid prior speed data";

        yield_decision->set_distance_s(-FLAGS_yield_distance);

        return 0;
    }

    if (hard_brake_speed_data->size() < 1 || soft_brake_speed_data->size() < 1)
    {
        AERROR << "No valid prior speed data";

        yield_decision->set_distance_s(-FLAGS_yield_distance);

        return 0;
    }

    // 计算车辆停车距离，期望的避让距离要大于stop dist
    // 但是，在其他车辆cut in时，lon dist往往比较小，所以使用soft brake
    // curve来计算s gap.
    double stop_dist;

    double k_dec = 2.5;

    stop_dist = adc_speed * adc_speed * 0.5 / k_dec;

    stop_dist = std::max(FLAGS_yield_distance, stop_dist);

    // obs s
    const auto& boundary = obstacle.path_st_boundary();

    const double obs_dist = boundary.min_s();

    // get most conservative follow dist

    const double adc_length =
            common::VehicleConfigHelper::GetConfig().vehicle_param().length();

    double ideal_yield_dist;

    ideal_yield_dist = stop_dist;

    double yield_dist;

    // follow_time_buffer , 2.5
    // follow_min_distance, 8

    yield_dist = std::fmax(ideal_yield_dist, FLAGS_yield_distance);

    // ego speed is small
    if (adc_speed < 0.1)
    {
        double ego_drive_dist = obs_dist - yield_dist;

        yield_decision->set_yield_acc_mode(YIELD_ACC_MODE_CONSTANT_SPEED);

        if (ego_drive_dist > 0.0)
        {
            yield_decision->set_distance_s(-yield_dist);

            return 0;
        }
        else
        {
            // 这里非常容易带来qp无解

            yield_decision->set_distance_s(-adc_length);

            return 0;
        }
    }

    estimate_yield_poi_by_two_phase_brake_curve(
            yield_decision, obstacle, adc_speed, adc_acc, path_length,
            hard_brake_speed_data, soft_brake_speed_data, stop_dist);

    return 0;
}

}
} // namespace apollo


