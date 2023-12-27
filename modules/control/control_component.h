#pragma once

#include <chrono>
#include <iostream>
#include <map>
#include <memory>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/control/common/dependency_injector.h"
#include "modules/control/controller/controller_agent.h"
#include "modules/control/proto/local_view.pb.h"
#include "cyber/node/node.h"
#include "cyber/time/time.h"

namespace apollo
{
namespace control
{

// this is control wrapper for apollo control module
class ControlComponent
{
public:
    ControlComponent();

    int init(std::string config_dir);

    // update data in per frame
    int update_cyber_rt();

    int process();

    apollo::localization::LocalizationEstimate *get_localization()
    {
        return local_view_.mutable_localization();
    }

    apollo::canbus::Chassis *get_chassis()
    {
        return local_view_.mutable_chassis();
    }

    apollo::planning::ADCTrajectory *get_trajectory()
    {
        return local_view_.mutable_trajectory();
    }

    int get_copy_trajectory(apollo::planning::ADCTrajectory *traj)
    {
        traj->CopyFrom(local_view_.trajectory());

        return 0;
    }

    int publish_msg();

    apollo::canbus::Chassis::DrivingMode get_chassis_drive_mode()
    {
        return latest_chassis_.driving_mode();
    }

    bool control_init_finish() { return init_finish_; }

    const apollo::control::ControlCommand &get_control_command()
    {
        return control_command_;
    }

protected:
    apollo::common::Status ProduceControlCommand(
            apollo::control::ControlCommand *control_command);

    apollo::common::Status CheckInput(apollo::control::LocalView *local_view);

    apollo::common::Status CheckTimestamp(
            const apollo::control::LocalView &local_view);
    apollo::common::Status CheckPad();

protected:
    apollo::cyber::Time init_time_;

    // input
    apollo::localization::LocalizationEstimate latest_localization_;
    apollo::canbus::Chassis latest_chassis_;
    apollo::planning::ADCTrajectory latest_trajectory_;
    // apollo::PadMessage pad_msg_;
    apollo::common::Header latest_replan_trajectory_header_;

    apollo::control::ControllerAgent controller_agent_;

    // output
    apollo::control::ControlCommand control_command_;

    bool estop_ = false;
    std::string estop_reason_;
    bool pad_received_ = false;

    unsigned int status_lost_ = 0;
    unsigned int status_sanity_check_failed_ = 0;
    unsigned int total_status_lost_ = 0;
    unsigned int total_status_sanity_check_failed_ = 0;

    apollo::control::ControlConf control_conf_;

    std::mutex mutex_;

    // reader
    std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>>
            chassis_reader_;

    std::shared_ptr<
            apollo::cyber::Reader<apollo::localization::LocalizationEstimate>>
            localization_reader_;

    std::shared_ptr<apollo::cyber::Reader<apollo::planning::ADCTrajectory>>
            trajectory_reader_;

    // writer
    std::shared_ptr<apollo::cyber::Writer<apollo::control::ControlCommand>>
            control_cmd_writer_;

    // when using control submodules
    std::shared_ptr<apollo::cyber::Writer<apollo::control::LocalView>>
            local_view_writer_;

    apollo::control::LocalView local_view_;

    std::shared_ptr<apollo::control::DependencyInjector> injector_;

    std::shared_ptr<apollo::cyber::Node> node_;

    bool init_finish_ = false;
};

}  // namespace control
}