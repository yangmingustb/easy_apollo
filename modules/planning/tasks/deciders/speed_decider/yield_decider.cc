
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

    YieldAccMode ego_acc_mode;
    yield_interaction_info hard_brake_interaction_info;
    yield_interaction_info soft_brake_interaction_info;

    double ego_yield_min_gap = 100.0;

    // double min_gap;
    double emergency_brake_curve_min_gap;
    double soft_brake_curve_min_gap;
    double end_poi_gap;

    // 计算point of interset, 包含起点，end point, min gap point

    // init
    yield_decision->set_distance_s(-FLAGS_yield_distance);

    yield_decision->mutable_yield_poi()->set_has_interaction(false);

    // update first phase
    estimate_st_gap_poi_by_st(&hard_brake_interaction_info, obstacle,
                              hard_brake_speed_data);

    if (hard_brake_interaction_info.has_has_interaction() &&
        hard_brake_interaction_info.min_gap_point().has_s_gap())
    {
        emergency_brake_curve_min_gap =
                hard_brake_interaction_info.min_gap_point().s_gap();
    }
    else
    {
        AERROR << "no st collision info";

        return 0;
    }

    if (emergency_brake_curve_min_gap <= 0.0)
    {
        ego_yield_min_gap = 1.0;
        ego_acc_mode = YIELD_ACC_MODE_HARD_BRAKE_AND_WILL_COLLISION;
    }
    else
    {
        estimate_st_gap_poi_by_st(&soft_brake_interaction_info, obstacle,
                                  soft_brake_speed_data);

        if (soft_brake_interaction_info.has_has_interaction() &&
            soft_brake_interaction_info.min_gap_point().has_s_gap())
        {
            soft_brake_curve_min_gap =
                    soft_brake_interaction_info.min_gap_point().s_gap();
        }
        else
        {
            AERROR << "no st collision info";

            return 0;
        }

        if (soft_brake_curve_min_gap <= 0.0)
        {
            ego_yield_min_gap = std::min(FLAGS_yield_distance,
                                         emergency_brake_curve_min_gap);

            ego_acc_mode = YIELD_ACC_MODE_HARD_BRAKE;
        }
        else if (soft_brake_curve_min_gap > emergency_brake_dist)
        {
            ego_yield_min_gap =
                    std::min(FLAGS_yield_distance, soft_brake_curve_min_gap);

            ego_acc_mode = YIELD_ACC_MODE_NOT_SURE;
        }
        else
        {
            ego_yield_min_gap =
                    std::min(FLAGS_yield_distance, soft_brake_curve_min_gap);

            ego_acc_mode = YIELD_ACC_MODE_SOFT_BRAKE;

            end_poi_gap = soft_brake_interaction_info.end_point().s_gap();

            end_poi_gap = std::min(end_poi_gap, FLAGS_yield_distance);

            yield_interaction_info* yield_poi =
                    yield_decision->mutable_yield_poi();

            yield_poi->set_has_interaction(true);

            yield_poi->mutable_end_point()->set_s_gap(end_poi_gap);

            yield_poi->mutable_end_point()->set_time(
                    soft_brake_interaction_info.end_point().time());
        }
    }

    yield_decision->set_yield_acc_mode(ego_acc_mode);
    yield_decision->set_distance_s(-ego_yield_min_gap);


#if debug_yield_decider

    const STBoundary& st_bound = obstacle.path_st_boundary();

    AINFO << "obs id: " << obstacle.Id()
          << ", yield acc mode: " << YieldAccMode_Name(ego_acc_mode)
          << ", yield min gap: " << ego_yield_min_gap
          << ", emergency_brake_dist " << emergency_brake_dist;

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

    // ego speed is not small
    // https://qwybs7wggx.feishu.cn/docx/NVBMdiKtJo3Qu4xJtnacrFran7b

    estimate_yield_poi_by_two_phase_brake_curve(
            yield_decision, obstacle, adc_speed, adc_acc, path_length,
            hard_brake_speed_data, soft_brake_speed_data, stop_dist);

    return 0;
}

int estimate_st_gap_poi_by_st(yield_interaction_info* interaction_info,
                              const Obstacle& obstacle,
                              const SpeedData* adc_speed_data)
{
    interaction_info->set_has_interaction(false);

    if (adc_speed_data == nullptr)
    {
        AERROR <<"speed data is null";

        return 0;
    }

    const STBoundary& st_bound = obstacle.path_st_boundary();

    if (adc_speed_data->size() < 1 || st_bound.IsEmpty())
    {
        AERROR << "speed data is invalid or no collision with obstacle";

        return 0;
    }

    double time;
    double unit_time = 0.1;

    double obs_s_upper;
    double obs_s_lower;

    double ego_s;
    common::SpeedPoint ego_speed_point;

    bool has_interaction;
    bool has_speed_point;

    double cur_gap;

    // init gap point

    double min_gap_point_t;
    double min_gap_s;
    
    double end_point_t;
    double end_point_s_gap;

    min_gap_point_t = 0.0;
    min_gap_s = 1000.0;

    bool has_first_interaction_point = false;

    // init
    time  = -0.1;

    for (size_t i = 0; i < 70; i++)
    {
        time += unit_time;

        has_interaction =
                st_bound.GetBoundarySRange(time, &obs_s_upper, &obs_s_lower);

        if (!has_interaction)
        {
            continue;
        }

        has_speed_point = adc_speed_data->EvaluateByTime(time, &ego_speed_point);
        if (!has_speed_point)
        {
            continue;
        }

        cur_gap = obs_s_lower - ego_speed_point.s();

        // update first point
        if (!has_first_interaction_point)
        {
            has_first_interaction_point = true;

            interaction_info->mutable_start_point()->set_time(time);
            interaction_info->mutable_start_point()->set_s_gap(cur_gap);
        }

        // update minimum gap point
        if (cur_gap < min_gap_s)
        {
            min_gap_s = cur_gap;
            min_gap_point_t = time;
        }

        // update end point
        end_point_t = time;
        end_point_s_gap = cur_gap;
    }

    interaction_info->set_has_interaction(has_first_interaction_point);

    interaction_info->mutable_min_gap_point()->set_time(min_gap_point_t);
    interaction_info->mutable_min_gap_point()->set_s_gap(min_gap_s);

    interaction_info->mutable_end_point()->set_time(end_point_t);
    interaction_info->mutable_end_point()->set_s_gap(end_point_s_gap);

#if debug_yield_decider


    AINFO << "obs id: " << obstacle.Id();
    AINFO << interaction_info->DebugString();

#endif

    return 0;
}

}
} // namespace apollo


