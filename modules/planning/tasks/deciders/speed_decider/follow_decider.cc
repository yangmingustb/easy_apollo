
#include "follow_decider.h"
#include "modules/planning/common/speed_profile_generator.h"


namespace apollo
{
namespace planning
{

#define debug_follow_decider (0)

int estimate_following_acc_mode(ObjectFollow* follow_decision,
                                const Obstacle& obstacle,
                                const double adc_speed, const double adc_acc,
                                const double path_length,
                                const SpeedData* hard_brake_speed_data,
                                const SpeedData* soft_brake_speed_data,
                                const double speed_limit)
{

    FollowingAccMode ego_follow_mode;
    follow_interaction_info hard_brake_interaction_info;
    follow_interaction_info soft_brake_interaction_info;

    double ego_follow_min_gap = 100.0;

    // double min_gap;
    double emergency_brake_curve_min_gap;
    double soft_brake_curve_min_gap;
    double end_point_gap;

    // init
    follow_decision->set_distance_s(-FLAGS_follow_min_distance);
    follow_decision->mutable_following_interaction_info()->set_has_intersection(
            false);

    estimate_min_lon_dist_gap_for_follow_decision(
            &hard_brake_interaction_info, obstacle, hard_brake_speed_data);

    if (hard_brake_interaction_info.has_has_intersection() &&
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
        ego_follow_min_gap = 1.0;
        ego_follow_mode = FollowingAccMode::HARD_BRAKE_AND_WILL_COLLISION;
    }
    else
    {
        estimate_min_lon_dist_gap_for_follow_decision(
                &soft_brake_interaction_info, obstacle, soft_brake_speed_data);

        if (soft_brake_interaction_info.has_has_intersection() &&
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
            ego_follow_min_gap = std::min(2.0, emergency_brake_curve_min_gap);

            ego_follow_mode = FollowingAccMode::HARD_BRAKE;
        }
        else
        {
            // 最小跟车距离
            ego_follow_min_gap = std::max(3.0, soft_brake_curve_min_gap);

            ego_follow_mode = FollowingAccMode::SOFT_BRAKE;

            // end point gap
            end_point_gap = soft_brake_interaction_info.end_point().s_gap();
            end_point_gap = std::max(2.0, end_point_gap);

            // time headway gap
            double obs_v = obstacle.speed();

            double time_headway ;
            if (obs_v < 11.1)
            {
                time_headway = 2.0;
            }
            else
            {
                time_headway = 3.0;
            }

            double expt_poi_gap = std::min(speed_limit, obs_v) * time_headway;
            expt_poi_gap = std::max(FLAGS_follow_min_distance, expt_poi_gap);

            // 
            expt_poi_gap = std::min(expt_poi_gap, end_point_gap);

            ego_follow_min_gap = std::min(ego_follow_min_gap, expt_poi_gap);

            follow_interaction_info* follow_poi =
                    follow_decision->mutable_following_interaction_info();

            follow_poi->set_has_intersection(true);

            follow_poi->mutable_end_point()->set_s_gap(expt_poi_gap);
            follow_poi->mutable_end_point()->set_time(
                    soft_brake_interaction_info.end_point().time());
        }
    }

    follow_decision->set_following_acc_mode(ego_follow_mode);
    follow_decision->set_distance_s(-ego_follow_min_gap);


#if debug_follow_decider

    const STBoundary& st_bound = obstacle.path_st_boundary();

    AINFO << "obs id: " << obstacle.Id()
          << ", follow mode: " << FollowingAccMode_Name(ego_follow_mode)
          << ", follow dist: " << ego_follow_min_gap
          << ", emergency_brake_dist " << emergency_brake_dist;

    AINFO << "start collision, t,s: " << st_bound.min_t() << " , "
          << st_bound.min_s();

    AINFO << follow_decision->DebugString();
#endif

    return 0;
}

// TODO(Jinyun): add more variables to follow gap calculation
int estimate_proper_following_gap(ObjectFollow *follow_decision,
                                     const Obstacle& obstacle,
                                     const double adc_speed,
                                     const double adc_acc,
                                     const double path_length,
                                     const SpeedData* hard_brake_speed_data,
                                     const SpeedData* soft_brake_speed_data,
                                     const SLBoundary& adc_sl_boundary,
                                     const double speed_limit)
{
    if (hard_brake_speed_data == nullptr || soft_brake_speed_data == nullptr)
    {
        AERROR << "No valid prior speed data";

        follow_decision->set_distance_s(-FLAGS_follow_min_distance);

        return 0;
    }

    if (hard_brake_speed_data->size() < 1 || soft_brake_speed_data->size() < 1)
    {
        AERROR << "No valid prior speed data";

        follow_decision->set_distance_s(-FLAGS_follow_min_distance);

        return 0;
    }

    double ideal_follow_dist;

    if (0)
    {
        // 计算车辆停车距离，跟车距离要大于stop dist,期望的跟车距离
        double stop_dist;

        double k_dec = 2.5;

        stop_dist = adc_speed * adc_speed * 0.5 / k_dec;

        stop_dist = std::max(FLAGS_follow_min_distance, stop_dist);

        ideal_follow_dist = stop_dist;
    }
    else
    {
        double obs_v = obstacle.speed();

        double ego_expt_v = std::min(obs_v, speed_limit);

        double time_headway;
        if (obs_v < 11.1)
        {
            time_headway = 2.0;
        }
        else
        {
            time_headway = 3.0;
        }

        // time headway
        ideal_follow_dist = ego_expt_v * time_headway;

        ideal_follow_dist =
                std::max(FLAGS_follow_min_distance, ideal_follow_dist);
    }

    // obs s
    const auto& boundary = obstacle.path_st_boundary();

    const double obs_dist = boundary.min_s();

    // get most conservative follow dist

    const double adc_length =
            common::VehicleConfigHelper::GetConfig().vehicle_param().length();


    // follow_time_buffer , 2.5
    // follow_min_distance, 8

    // ego speed is small
    if (adc_speed < 0.1)
    {
        double ego_drive_dist = obs_dist - ideal_follow_dist;
        if (ego_drive_dist > 0.0)
        {
            follow_decision->set_distance_s(-ideal_follow_dist);

            return 0;
        }
        else
        {
            // 这里非常容易带来qp无解

            follow_decision->set_distance_s(-adc_length);

            return 0;
        }
    }

    // ego speed is not small
    // https://qwybs7wggx.feishu.cn/docx/NVBMdiKtJo3Qu4xJtnacrFran7b

    estimate_following_acc_mode(follow_decision, obstacle, adc_speed, adc_acc,
                                path_length, hard_brake_speed_data,
                                soft_brake_speed_data, speed_limit);

    return 0;
}

double estimate_min_lon_dist_gap_for_follow_decision(
        const Obstacle& obstacle, const SpeedData& adc_speed_data)
{
    double time;
    double unit_time = 0.1;

    // init
    time  = -0.1;

    const STBoundary& st_bound = obstacle.path_st_boundary();

    double obs_s_upper;
    double obs_s_lower;

    double ego_s;
    common::SpeedPoint ego_speed_point;

    bool has_interaction;
    bool has_speed_point;

    double cur_gap;
    double min_gap = 1000.0;

    for (size_t i = 0; i < 70; i++)
    {
        time += unit_time;

        has_interaction =
                st_bound.GetBoundarySRange(time, &obs_s_upper, &obs_s_lower);

        if (!has_interaction)
        {
            continue;
        }

        has_speed_point = adc_speed_data.EvaluateByTime(time, &ego_speed_point);
        if (!has_speed_point)
        {
            continue;
        }

        cur_gap = obs_s_lower - ego_speed_point.s();

        if (cur_gap < min_gap)
        {
            min_gap = cur_gap;
        }
    }

    return min_gap;
}

int estimate_min_lon_dist_gap_for_follow_decision(
        follow_interaction_info* interaction_info, const Obstacle& obstacle,
        const SpeedData* adc_speed_data)
{
    interaction_info->set_has_intersection(false);

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

    interaction_info->set_has_intersection(has_first_interaction_point);

    interaction_info->mutable_min_gap_point()->set_time(min_gap_point_t);
    interaction_info->mutable_min_gap_point()->set_s_gap(min_gap_s);

    interaction_info->mutable_end_point()->set_time(end_point_t);
    interaction_info->mutable_end_point()->set_s_gap(end_point_s_gap);

#if debug_follow_decider


    AINFO << "obs id: " << obstacle.Id();
    AINFO << interaction_info->DebugString();

#endif

    return 0;
}

}
} // namespace apollo


