
#include "modules/planning/common/speed_profile_generator.h"

#include "stop_decider.h"


namespace apollo
{

namespace planning
{

#define debug_stop_decider (0)

// https://qwybs7wggx.feishu.cn/docx/DIgidRdAjookncxs0AccHv3snVg
int estimate_stop_dist_for_reverse_driving_obs(double* path_efence_s,
                                               const Obstacle& obstacle,
                                               const double adc_speed)
{


    // obs s
    const auto& boundary = obstacle.path_st_boundary();

    double obs_dist = boundary.max_s();

    double speed_obs = obstacle.speed();

    double combine_speed = 0.5 * adc_speed + speed_obs;

    const common::VehicleParam& veh_param =
            common::VehicleConfigHelper::GetConfig().vehicle_param();

    const double adc_length = veh_param.length();

    double front_edge_to_center = veh_param.front_edge_to_center();

    double ideal_stop_interval = 6.0;

    double ego_collision_s = boundary.max_s();

    obs_dist += front_edge_to_center;


    // obs speed is low, 将efence 放到obstacle附近即可
    if (speed_obs < 0.1 || ego_collision_s < 0.5)
    {
        double ego_front_dist = obs_dist - ideal_stop_interval;

        if (ego_front_dist > front_edge_to_center)
        {
            *path_efence_s = ego_front_dist;

#if debug_stop_decider
            AINFO << "ego_front_dist: " << ego_front_dist;
#endif
            return 0;
        }
        else
        {
            *path_efence_s = adc_length;

#if debug_stop_decider
            AINFO << "adc_length: " << adc_length;
#endif

            return 0;
        }
    }

    // if ego speed is low, 将efence 放到ego附近即可

    if (adc_speed < 0.1)
    {
        double obs_min_dist = boundary.min_s() + front_edge_to_center;

        *path_efence_s = std::max(adc_length, obs_min_dist);
        return 0;
    }

    // 估计减速时间
    double constant_dec_time;

    constant_dec_time = ego_collision_s / combine_speed;

    //估计dec
    double constant_dec = -adc_speed / constant_dec_time;

    constant_dec = std::max(constant_dec, veh_param.max_deceleration());
    // 估计后轴stop dist
    double stop_dist;

    stop_dist = -adc_speed * adc_speed * 0.5 / constant_dec;

    stop_dist += front_edge_to_center;

    // 增加一个额外的安全距离，不要加上车长
    *path_efence_s = std::max(stop_dist, adc_length);

    // 

    #if debug_stop_decider
    AINFO << "stop_dist: " << stop_dist << " , adc_length " << adc_length;
#endif


    return 0;
}

}  // namespace planning

} // namespace apollo
