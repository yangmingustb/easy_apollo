#include "modules/viz2d/viz_window.h"

#include "modules/chassis/virtual_chassis.h"
#include "modules/viz2d/display_config.pb.h"

#include "modules/control/control_component.h"
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
    // double curr_time_stamp = TimeUtils::GetTimeStamp() / 1000;
    // apollo::cyber::Clock::SetMode(apollo::cyber::proto::MODE_MOCK);
    // apollo::cyber::Clock::SetNowInSeconds(curr_time_stamp);
    // todo, 将来所有数据的时钟都需要改称apollo cyber时钟

    // double history_time = curr_time_stamp;

    google::ParseCommandLineFlags(&argc, &argv, true);

    // module state
    SystemState module_state;
    SystemStateInit(module_state);

    std::string base_dir = "./../modules";
    viz2d_window_read_config(base_dir);

    apollo::cyber::binary::SetName("control");

    // init cyber framework
    apollo::cyber::Init(apollo::cyber::binary::GetName().c_str());

    apollo::control::ControlComponent control;

    control.init(base_dir);

    FLAGS_vehicle_config_path =
            "./../modules/common/data/vehicle_param.pb.txt";

    common::VehicleConfig vehicle_config_;
    vehicle_config_ = apollo::common::VehicleConfigHelper::GetConfig();

    FLAGS_vehicle_model_config_filename =
            "./../modules/common/vehicle_model/conf/"
            "vehicle_model_config.pb.txt";


    int ret_value;


    int key_value;

    double interval_time = 20.0;

    // 10 ms
    double history_time;
    double current_time;
    double delta_time;
    history_time = apollo::cyber::Time::Now().ToSecond();

    bool init_finish = control.control_init_finish();

    running_time_debug_info();

    while (apollo::cyber::OK())
    {
        if (!init_finish)
        {
            AINFO << "init waiting";
            continue;
        }

        double cur_time = apollo::cyber::Time::Now().ToSecond();
        delta_time = (cur_time - history_time) * 1000.0;

        if (delta_time < interval_time)
        {
            continue;
        }

        int ret_value;
        float lat_cmd = 0.0;
        float lon_cmd = 0.0;

        control.update_cyber_rt();

        apollo::control::ControlCommand history_command =
                control.get_control_command();

        apollo::canbus::Chassis *latest_chassis_ = control.get_chassis();

        {
            ret_value = control.process();
        }

        if (ret_value < 0)
        {
            if (ret_value == -2)
            {
                AERROR << "control no trajectory\n";
            }
            else if (ret_value == -3)
            {
                AERROR << "null input for control\n";
            }
            else if (ret_value == -4)
            {
                AERROR << "chassis error for control\n";
            }
            else
            {
                AERROR << "control error\n";
            }
        }
        else
        {
            control.publish_msg();
        }

        AINFO << "control frame interval (ms): " << delta_time;

        const apollo::RunningTimeDebug *debug = apollo::get_debug_info();

        if (debug->pause_debug.enabled)
        {
            continue;
        }
        history_time = cur_time;
    }

    // terminate
    apollo::cyber::Clear();

    google::ShutDownCommandLineFlags();

    return 0;
}
