#include "viz3d_component.h"
#include "modules/common/configs/vehicle_config_helper.h"

using namespace apollo;

// 和替他模块不同，每一帧循环的时间依赖cyber rt, viz3d的循环依赖open
// gl本身的计时器
int main(int argc, char** argv)
{
    double cur_time = apollo::cyber::Time::Now().ToSecond();

    // apollo::cyber::Clock::SetNowInSeconds(curr_time_stamp);
    // todo, 将来所有数据的时钟都需要改称apollo cyber时钟

    google::ParseCommandLineFlags(&argc, &argv, true);

    apollo::cyber::binary::SetName("viewer");

    // init cyber framework
    apollo::cyber::Init(apollo::cyber::binary::GetName().c_str());

    // module state
    SystemState module_state;
    SystemStateInit(module_state);

    // init apollo module
    std::string base_dir = "./../modules";


    FLAGS_vehicle_config_path =
            "./../modules/common/data/vehicle_param.pb.txt";

    FLAGS_vehicle_model_config_filename = "./../modules/common/vehicle_model/"
                                          "conf/vehicle_model_config.pb.txt";


    viz3d_component viz3d_;
    viz3d_.init();

    // reader

    // writter

    int ret_value;

    running_time_debug_info();

    // vis

    int key_value;

    // 100毫秒一个周期
    double interval_time = 50.0;
    double history_time;
    double current_time;
    double delta_time;
    history_time = apollo::cyber::Time::Now().ToSecond();

    unsigned int mode;

    // veh params
    apollo::common::VehicleConfig vehicle_config =
            apollo::common::VehicleConfigHelper::GetConfig();

    const apollo::common::VehicleParam& apollo_veh =
            vehicle_config.vehicle_param();

    double max_front_wheel_angle;
    max_front_wheel_angle =
            apollo_veh.max_steer_angle() / apollo_veh.steer_ratio();

    double max_steering_wheel_angle_ = apollo_veh.max_steer_angle();

    max_steering_wheel_angle_ = max_steering_wheel_angle_ * 180 / M_PI;

    while (apollo::cyber::OK())
    {
        double cur_time = apollo::cyber::Time::Now().ToSecond();
        delta_time = (cur_time - history_time) * 1000.0;

        if (delta_time < interval_time)
        {
            continue;
        }

        AINFO << "frame interval (ms): " << delta_time;

        viz3d_.process(max_steering_wheel_angle_);
        viz3d_.refresh_in_per_frame();

        const apollo::RunningTimeDebug *debug = apollo::get_debug_info();

        if (debug->pause_debug.enabled)
        {
            continue;
        }
        history_time = cur_time;
    }

    // terminate
    viz3d_.close();
    apollo::cyber::Clear();

    google::ShutDownCommandLineFlags();

    return 0;
}
