#include "modules/viz2d/viz_window.h"
#include "viz2d_component.h"
#include "modules/viz2d/display_config.pb.h"
#include "modules/chassis/virtual_chassis.h"
#include "modules/common/status/system_state.h"
#include "modules/common/configs/vehicle_config_helper.h"

using namespace apollo;
using namespace apollo::prediction;
using namespace apollo::common;
using namespace apollo::routing;
using apollo::cyber::Rate;

/** virutal localization
 * virtual chasis
 *
 * planner get virtual localization,virtual chasis;
 */

int main(int argc, char** argv)
{
    // apollo::cyber::Clock::SetMode(apollo::cyber::proto::MODE_MOCK);
    // apollo::cyber::Clock::SetNowInSeconds(curr_time_stamp);
    // todo, 将来所有数据的时钟都需要改称apollo cyber时钟

    google::ParseCommandLineFlags(&argc, &argv, true);

    apollo::cyber::binary::SetName("display2d");

    // init cyber framework
    apollo::cyber::Init(apollo::cyber::binary::GetName().c_str());

    // module state
    SystemState module_state;
    SystemStateInit(module_state);

    // init apollo module
    std::string base_dir = "./../modules";
    viz2d_window_read_config(base_dir);


    FLAGS_vehicle_config_path =
            "./../modules/common/data/vehicle_param.pb.txt";


    FLAGS_vehicle_model_config_filename = "./../modules/common/vehicle_model/conf/vehicle_model_config.pb.txt";

    // reader

    // writter

    int ret_value;

    running_time_debug_info();

    // vis
    apollo::viz2d_component viz2d;
    viz2d.init();

    int key_value;

    // 10毫秒一个周期
    double interval_time = 20.0;
    double history_time;
    double current_time;
    double delta_time;
    history_time = apollo::cyber::Time::Now().ToSecond();

    common::VehicleConfig vehicle_config_;

    //
    vehicle_config_ = apollo::common::VehicleConfigHelper::GetConfig();

    double max_steering_wheel_angle_ =
            vehicle_config_.vehicle_param().max_steer_angle();

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

        viz2d.process(max_steering_wheel_angle_);

        const apollo::RunningTimeDebug *debug = apollo::get_debug_info();

        if (debug->pause_debug.enabled)
        {
            continue;
        }
        history_time = cur_time;
    }

    // terminate
    viz2d.close();
    apollo::cyber::Clear();

    google::ShutDownCommandLineFlags();

    return 0;
}
