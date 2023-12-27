
#include "chassis_component.h"
#include "modules/common/adapters/adapter_gflags.h"

namespace apollo
{

#define manual_mode_deceleration (-1.0)

ChassisComponent::ChassisComponent(/* args */)
{
    node_ = apollo::cyber::CreateNode(apollo::cyber::binary::GetName());
}

int ChassisComponent::init()
{
    chassis_writer_ = node_->CreateWriter<canbus::Chassis>(FLAGS_chassis_topic);
    localization_writer_ =
            node_->CreateWriter<localization::LocalizationEstimate>(
                    FLAGS_localization_topic);

    obs_writer_ = node_->CreateWriter<apollo::perception::PerceptionObstacles>(
            FLAGS_perception_obstacle_topic);

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


    control_command_.Clear();

    // set initial vehicle state by cmd
    // need to sleep, because advertised channel is not ready immediately
    // simple test shows a short delay of 80 ms or so
    AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return 0;
}

int ChassisComponent::publish_chassis(
        std::shared_ptr<apollo::canbus::Chassis> &chassis)
{
    if (drive_mode_.driving_mode() == apollo::canbus::Chassis::COMPLETE_MANUAL)
    {
        chassis->set_driving_mode(apollo::canbus::Chassis::COMPLETE_MANUAL);
    }
    else
    {
        chassis->set_driving_mode(apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE);
    }

    if (chassis->has_header())
    {
        chassis_writer_->Write(chassis);
    }
    else
    {
        AERROR << "chassis no header";
    }

    return 0;
}

int ChassisComponent::publish_localization(
        std::shared_ptr<apollo::localization::LocalizationEstimate>
                &ptr_localization)
{
    if (ptr_localization->has_header())
    {
        localization_writer_->Write(ptr_localization);
    }
    else
    {
        AERROR << "localization no header";
    }

    return 0;
}

int ChassisComponent::publish_perception(
        std::shared_ptr<apollo::perception::PerceptionObstacles> &preception)
{
    if (preception->has_header())
    {
        obs_writer_->Write(preception);
    }
    else
    {
        AERROR << "perception no header";
    }
    return 0;
}

int ChassisComponent::process(std::vector<float> &chassis_can_msg)
{

    {
        std::lock_guard<std::mutex> lock(mutex_);

        local_view_.latest_control_command.Clear();
        local_view_.latest_control_command.CopyFrom(control_command_);
    }

    if (!local_view_.latest_control_command.has_header())
    {
        AERROR << "No header";
        return 0;
    }
    if (!local_view_.latest_control_command.has_acceleration())
    {
        AERROR << "No acceleration";
        return 0;
    }
    if (std::isnan(local_view_.latest_control_command.acceleration()))
    {
        AERROR << "acceleration is nan";
        return 0;
    }

    if (!local_view_.latest_control_command.has_steering_target())
    {
        AERROR << "No steering_target";
        return 0;
    }
    if (std::isnan(local_view_.latest_control_command.steering_target()))
    {
        AERROR << "steering_target is nan";
        return 0;
    }

    // 将apollo格式的指令转换成内部格式
    apollo_control_msg_to_chassis_can_msg(chassis_can_msg,
                                          local_view_.latest_control_command);

    // 只有是手动模式,才会减速.
    // 其他情形,则使用control的acc.
    if (drive_mode_.driving_mode() ==
        apollo::canbus::Chassis::DrivingMode::
                Chassis_DrivingMode_COMPLETE_MANUAL)
    {
        chassis_can_msg[1] = manual_mode_deceleration;

        AINFO << "manual mode, so publish brake command";
    }

    return 0;
}
}  // namespace apollo