#pragma once
#include <memory>
#include <string>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/time/time.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/common/util/util.h"
#include "modules/control/common/dependency_injector.h"
#include "modules/control/controller/controller_agent.h"
#include "modules/control/proto/preprocessor.pb.h"
#include "modules/control/submodules/preprocessor_submodule.h"

#include "modules/common/configs/vehicle_config_helper.h"

#include "virtual_chassis.h"
#include "chassis_local_view.h"

namespace apollo
{

// 真实底盘
class ChassisComponent
{
private:

    // hmi发送的底盘状态
    canbus::Chassis drive_mode_;

    // cyber rt data
    control::ControlCommand control_command_;

    chassis_local_view local_view_;

    // writter
    std::shared_ptr<cyber::Writer<apollo::canbus::Chassis>> chassis_writer_;
    std::shared_ptr<cyber::Writer<apollo::localization::LocalizationEstimate>>
            localization_writer_;

    std::shared_ptr<
            apollo::cyber::Writer<apollo::perception::PerceptionObstacles>>
            obs_writer_;

    // reader

    std::shared_ptr<cyber::Reader<apollo::control::ControlCommand>> control_reader_;
    std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> drive_mode_reader_;

    // node
    std::shared_ptr<apollo::cyber::Node> node_;

    std::mutex mutex_;

public:
    ChassisComponent(/* args */);
    ~ChassisComponent(){};

    int init();

    int publish_localization(
            std::shared_ptr<apollo::localization::LocalizationEstimate>
                    &ptr_localization);

    int publish_perception(
            std::shared_ptr<apollo::perception::PerceptionObstacles>
                    &preception);

    int publish_chassis(
            std::shared_ptr<apollo::canbus::Chassis>
                    &chassis);

    int process(std::vector<float> &chassis_can_msg);

    int apollo_control_msg_to_chassis_can_msg(
            std::vector<float> &chassis_can_msg,
            const apollo::control::ControlCommand &control_command)
    {
        const apollo::common::VehicleConfig &vehicle_config =
                apollo::common::VehicleConfigHelper::GetConfig();

        // 方向盘转角
        double steering_wheel_angle;
        steering_wheel_angle = control_command.steering_target() / 100 *
                               vehicle_config.vehicle_param().max_steer_angle();

        steering_wheel_angle = steering_wheel_angle * 180.0 / M_PI;

        // 注意：这里和读取时候的符号不一致.请参考读取的注释.
        // set virtual cmd
        // 指令发送时：
        // apollo,左转为正.
        // 底盘控制器，向右为正
        chassis_can_msg.push_back(-steering_wheel_angle);
        chassis_can_msg.push_back(control_command.acceleration());


        return 0;
    }
};

}  // namespace apollo