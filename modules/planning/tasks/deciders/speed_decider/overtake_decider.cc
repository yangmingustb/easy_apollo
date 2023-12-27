#include "modules/planning/tasks/deciders/speed_decider/overtake_decider.h"

namespace apollo
{

namespace planning
{

int estimate_min_lon_gap_for_overtake_decision(double* min_gap,
                                               const Obstacle& obstacle,
                                               const SpeedData* adc_speed_data)
{
    *min_gap = 1000.0;

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


    double min_gap_point_t;
    double min_gap_s;
    

    min_gap_point_t = 0.0;
    min_gap_s = 1000.0;


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

        cur_gap = ego_speed_point.s() - obs_s_upper;


        // update minimum gap point
        if (cur_gap < min_gap_s)
        {
            min_gap_s = cur_gap;
            min_gap_point_t = time;
        }

        if (min_gap_s < 0.1)
        {
            *min_gap = 0.0;
            return 0;
        }

    }


    return 0;
}

}
}  // namespace apollo