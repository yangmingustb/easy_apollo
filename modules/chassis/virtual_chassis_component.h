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

#include "virtual_chassis.h"
#include "chassis_local_view.h"

namespace apollo
{

//虚拟底盘
class virtual_chassis_interface
{
private:
    // pub
    std::shared_ptr<localization::LocalizationEstimate> latest_localization_;
    // pub
    std::shared_ptr<canbus::Chassis> latest_chassis_;

    // hmi发送的底盘状态
    // receive
    canbus::Chassis drive_mode_;
    // receive
    apollo::cyber::proto::DebugMsg debug_msg_;

    // apollo 格式的控制
    // receive
    control::ControlCommand control_command_;

    // 内部格式的控制
    CanCommond virtual_chassis_cmd_;

    // writter
    std::shared_ptr<cyber::Writer<apollo::canbus::Chassis>> chassis_writer_;
    std::shared_ptr<cyber::Writer<apollo::localization::LocalizationEstimate>>
            localization_writer_;

    // reader

    std::shared_ptr<cyber::Reader<apollo::control::ControlCommand>>
            control_reader_;
    std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> drive_mode_reader_;

    std::shared_ptr<apollo::cyber::Reader<apollo::cyber::proto::DebugMsg>>
            debug_msg_reader_;

    // node
    std::shared_ptr<apollo::cyber::Node> node_;

    std::mutex mutex_;

    VirtualChassis chassis_;

    chassis_local_view local_view_;

public:
    virtual_chassis_interface(/* args */);
    ~virtual_chassis_interface(){};

    int init(const apollo::common::VehicleParam &veh_param,
             apollo::localization::Pose &start_point, double cycle_time);

    int publish_msg();

    int process();
};

}  // namespace apollo