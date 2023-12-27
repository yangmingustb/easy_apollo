
#include "virtual_chassis_component.h"
#include "modules/common/adapters/adapter_gflags.h"

namespace apollo
{

// 用来测试底盘不响应的case，观察规控.
// 0: 底盘不响应
// 1： 底盘响应
#define chassis_response_debug (1)

virtual_chassis_interface::virtual_chassis_interface(/* args */)
{
    node_ = apollo::cyber::CreateNode(apollo::cyber::binary::GetName());
}

int virtual_chassis_interface::init(
        const apollo::common::VehicleParam &veh_param,
        apollo::localization::Pose &start_point, double cycle_time)
{
    chassis_writer_ = node_->CreateWriter<canbus::Chassis>(FLAGS_chassis_topic);
    localization_writer_ =
            node_->CreateWriter<localization::LocalizationEstimate>(
                    FLAGS_localization_topic);

    control_reader_ = node_->CreateReader<control::ControlCommand>(
            FLAGS_control_command_topic,
            [this](const std::shared_ptr<control::ControlCommand>
                           &control_command) {
                ADEBUG << "Received planning data: run planning callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                control_command_.CopyFrom(*control_command);
            });

    drive_mode_reader_ = node_->CreateReader<apollo::canbus::Chassis>(
            FLAGS_chassis_drive_mode_topic,
            [this](const std::shared_ptr<apollo::canbus::Chassis>
                           &drive_mode) {
                ADEBUG << "Received planning data: run planning callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                drive_mode_.CopyFrom(*drive_mode);
            });

    debug_msg_reader_ = node_->CreateReader<cyber::proto::DebugMsg>(
            FLAGS_debug_planning_msg,
            [this](const std::shared_ptr<cyber::proto::DebugMsg> &debug)
            {
                ADEBUG << "Received perception data: run perception callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                debug_msg_.CopyFrom(*debug);
            });

    latest_localization_ =
            std::make_shared<localization::LocalizationEstimate>();
    latest_chassis_ = std::make_shared<canbus::Chassis>();
    latest_chassis_->set_driving_mode(
            apollo::canbus::Chassis_DrivingMode::
                    Chassis_DrivingMode_COMPLETE_AUTO_DRIVE);

    chassis_.Init(veh_param);

    chassis_.SetInitChassisState(
            start_point.position().x(), start_point.position().y(),
            start_point.position().z(), start_point.heading());

    chassis_.SetCommondTimeInterval(cycle_time);

    // set initial vehicle state by cmd
    // need to sleep, because advertised channel is not ready immediately
    // simple test shows a short delay of 80 ms or so
    AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return 0;
}

int virtual_chassis_interface::publish_msg()
{
    double timestamp = apollo::cyber::Clock::NowInSeconds();
    chassis_.PublishChassis(timestamp, latest_chassis_);

    if (drive_mode_.driving_mode() == apollo::canbus::Chassis::COMPLETE_MANUAL)
    {
        latest_chassis_->set_driving_mode(
                apollo::canbus::Chassis::COMPLETE_MANUAL);
    }
    else
    {
        latest_chassis_->set_driving_mode(
                apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE);
    }

    chassis_.PublishLocalization(latest_localization_, timestamp);

    // latest_chassis_->set_speed_mps(20.0);

    chassis_writer_->Write(latest_chassis_);

    localization_writer_->Write(latest_localization_);

    // debug
#if 0
    AINFO << latest_localization_->DebugString();
#endif

    return 0;
}

int virtual_chassis_interface::process()
{
    bool is_control_cmd_valid = true;

    {
        std::lock_guard<std::mutex> lock(mutex_);

        local_view_.latest_control_command.Clear();
        local_view_.latest_control_command.CopyFrom(control_command_);
    }

    if (!local_view_.latest_control_command.has_header())
    {
        AERROR << "No header";
        is_control_cmd_valid = false;
    }
    if (!local_view_.latest_control_command.has_acceleration())
    {
        AERROR << "No acceleration";
        is_control_cmd_valid = false;
    }

    if (!local_view_.latest_control_command.has_steering_target())
    {
        AERROR << "No steering_target";
        is_control_cmd_valid = false;
    }

    // cmd有效会更新virtual_chassis_cmd_， 否则保持默认状态行驶
    if (is_control_cmd_valid)
    {
        // 将apollo格式的指令转换成内部格式
        apollo_control_msg_to_virtual_chassis_control_msg(
                &virtual_chassis_cmd_, local_view_.latest_control_command);
    }

    // 只有是手动模式,才会减速.
    // 其他情形,则使用control的acc.
    if (drive_mode_.driving_mode() ==
        apollo::canbus::Chassis::DrivingMode::
                Chassis_DrivingMode_COMPLETE_MANUAL)
    {
        virtual_chassis_cmd_.acc_ = -0.3;
    }

    if (debug_msg_.debug_mode() != cyber::proto::DebugMode::none)
    {
        AINFO << "debug mode:pause";
        return 0;
    }

    chassis_.SetInput(virtual_chassis_cmd_);

#if chassis_response_debug
    chassis_.Process();
#endif

    return 0;
}
}  // namespace apollo