#include "virtual_chassis.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/math/pose.h"

#include "modules/common/configs/vehicle_config_helper.h"

#include "modules/common/util/message_util.h"

namespace apollo
{
using namespace apollo;

VirtualChassis::VirtualChassis(/* args */) {}

VirtualChassis::~VirtualChassis() {}

int VirtualChassis::PerfectResponse()
{
    /* update steer */
    state_.steer_ = cmd_.steer_;

    /* update velocity */

    double updated_vel = std::fabs(state_.v_);
    updated_vel += cmd_.acc_ * cmd_interval_;

    updated_vel = std::max(0.0, updated_vel);

    state_.v_ = updated_vel;

    state_.acc_ = cmd_.acc_;

    return 0;
}

int VirtualChassis::chassis_response_by_action_delay()
{
    double delay_time = 0.2;

    /* update steer */

    double delta_steer = cmd_.steer_ - history_state_.steer_;

    delta_steer = delta_steer * cmd_interval_ / delay_time;

    state_.steer_ = history_state_.steer_ + delta_steer;

    /* update velocity */
    delay_time = 0.1;
    double updated_vel = std::fabs(state_.v_);
    updated_vel += cmd_.acc_ * cmd_interval_;
    updated_vel = std::max(0.0, updated_vel);

    double delta_speed = updated_vel - history_state_.v_;

    if (delay_time < 0.02)
    {
        delta_speed = delta_speed * cmd_interval_ / delay_time;
    }

    state_.v_ = history_state_.v_ + delta_speed;

    state_.acc_ = cmd_.acc_;

    return 0;
}

int VirtualChassis::chassis_response_by_action_delay_and_overshoot()
{
    double delay_time = 0.2;

    /* update steer */

    double overshoot = 2.0 * M_PI / 180.0;

    // 超调2度
    double cmd_steer = cmd_.steer_;
    if (cmd_steer > 0.0)
    {
        cmd_steer += overshoot;
    }
    else if (cmd_steer < 0.0)
    {
        cmd_steer -= overshoot;
    }

    double delta_steer = cmd_steer - history_state_.steer_;

    delta_steer = delta_steer * cmd_interval_ / delay_time;

    state_.steer_ = history_state_.steer_ + delta_steer;

    /* update velocity */
    delay_time = 0.1;
    double updated_vel = std::fabs(state_.v_);
    updated_vel += cmd_.acc_ * cmd_interval_;
    updated_vel = std::max(0.0, updated_vel);

    double delta_speed = updated_vel - history_state_.v_;

    delta_speed = delta_speed * cmd_interval_ / delay_time;

    state_.v_ = history_state_.v_ + delta_speed;

    state_.acc_ = cmd_.acc_;

    return 0;
}

double VirtualChassis::calc_turn_radius(double lfr, double steering)
{
    double rad = steering;

    double radius = lfr / std::tan(rad);

    return radius;
}

int VirtualChassis::UpdateChassisPose(Pose2D *veh_pose, double steering_angle,
                                      double move_dist,
                                      apollo::common::VehicleParam *veh_params)
{
    double radius;
    Pose2D center_pos;

    if (std::fabs(steering_angle) > 0.00001)
    {
        radius = calc_turn_radius(veh_params->wheel_base(), steering_angle);

        center_pos.pos.x = veh_pose->pos.x - radius * std::sin(veh_pose->theta);
        center_pos.pos.y = veh_pose->pos.y + radius * std::cos(veh_pose->theta);

        veh_pose->theta += move_dist / radius;
        veh_pose->pos.x = center_pos.pos.x + radius * std::sin(veh_pose->theta);
        veh_pose->pos.y = center_pos.pos.y - radius * std::cos(veh_pose->theta);
    }
    else
    {
        veh_pose->pos.x += move_dist * std::cos(veh_pose->theta);
        veh_pose->pos.y += move_dist * std::sin(veh_pose->theta);
    }

    return 0;
}

int VirtualChassis::Process()
{
    history_state_ = state_;

    chassis_update_type type = chassis_update_type::time_delay;
    if (type == chassis_update_type::perfect_response)
    {
        PerfectResponse();
    }
    else if (type == chassis_update_type::time_delay)
    {
        chassis_response_by_action_delay();
    }
    else
    {
        chassis_response_by_action_delay_and_overshoot();
    }

    Pose2D cur_pose;
    cur_pose.pos.x = state_.x_;
    cur_pose.pos.y = state_.y_;
    cur_pose.theta = state_.heading_;

    double move_dist = state_.v_ * cmd_interval_;
    UpdateChassisPose(&cur_pose, state_.steer_, move_dist, &veh_params_);

    state_.x_ = cur_pose.pos.x;
    state_.y_ = cur_pose.pos.y;
    // state_.heading_ = cur_pose.heading;

    double theta = apollo_unify_theta(cur_pose.theta, apollo_PI);
    state_.heading_ = theta;

    if (debug_virtual_chassis)
    {
        printf("chassis state, steer %.2f, v %.2f,acc %.2f, x %.2f, y %.2f, "
               "heading %.2f,cmd time %.4f \n",
               state_.steer_ * 180 / M_PI, state_.v_, state_.acc_, state_.x_,
               state_.y_, state_.heading_, cmd_interval_);
    }
    return 0;
}

int VirtualChassis::PublishChassis(
        double curr_time_stamp,
        std::shared_ptr<apollo::canbus::Chassis> &ptr_chassis)
{
    if (ptr_chassis == nullptr)
    {
        std::cout << "/* ptr is null */" << std::endl;

        return -1;
    }

    // set
    ptr_chassis->mutable_header()->set_timestamp_sec(curr_time_stamp);
    ptr_chassis->set_engine_started(true);
    ptr_chassis->set_engine_rpm(0);
    ptr_chassis->set_odometer_m(0);
    ptr_chassis->set_fuel_range_m(0);
    ptr_chassis->set_parking_brake(false);
    ptr_chassis->set_error_code(
            apollo::canbus::Chassis_ErrorCode::Chassis_ErrorCode_NO_ERROR);
    ptr_chassis->set_gear_location(apollo::canbus::Chassis_GearPosition::
                                           Chassis_GearPosition_GEAR_DRIVE);

    apollo::common::VehicleSignal *v_signal = ptr_chassis->mutable_signal();
    v_signal->set_turn_signal(apollo::common::VehicleSignal_TurnSignal::
                                      VehicleSignal_TurnSignal_TURN_NONE);
    v_signal->set_horn(false);

    float speed_mps = state_.v_;
    float throttle_percentage = -1.0;
    float brake_percentage = -1.0;
    double front_wheel_angle = state_.steer_;

    apollo::common::VehicleConfig vehicle_config =
            apollo::common::VehicleConfigHelper::GetConfig();

    double steering_wheel_angle =
            front_wheel_angle * vehicle_config.vehicle_param().steer_ratio();

    float steering_percentage =
            steering_wheel_angle /
            vehicle_config.vehicle_param().max_steer_angle();

    steering_percentage *= 100;

    float steering_torque_nm = -999999;

    ptr_chassis->set_speed_mps(speed_mps < 0 ? 0 : speed_mps);
    ptr_chassis->set_throttle_percentage(
            throttle_percentage < 0 ? 0 : throttle_percentage);
    ptr_chassis->set_brake_percentage(brake_percentage < 0 ? 0
                                                           : brake_percentage);
    ptr_chassis->set_steering_percentage(
            steering_percentage < -10000 ? 0 : steering_percentage);
    ptr_chassis->set_steering_torque_nm(
            steering_torque_nm < -10000 ? 0 : steering_torque_nm);

    return 0;
}

void VirtualChassis::PublishLocalization(
        std::shared_ptr<localization::LocalizationEstimate> &localization,
        double curr_time_stamp)
{
    apollo::common::util::FillHeader("localization", localization.get());

    localization->mutable_header()->set_timestamp_sec(curr_time_stamp);
    auto *pose = localization->mutable_pose();

    // Set position
    pose->mutable_position()->set_x(state_.x_);
    pose->mutable_position()->set_y(state_.y_);
    pose->mutable_position()->set_z(state_.z_);

    // Set orientation and heading
    double cur_theta = state_.heading_;

    // if (FLAGS_use_navigation_mode)
    // {
    //     double flu_x = point.path_point().x();
    //     double flu_y = point.path_point().y();

    //     Eigen::Vector2d enu_coordinate =
    //             common::math::RotateVector2d({flu_x, flu_y}, cur_theta);

    //     enu_coordinate.x() += adc_position_.x();
    //     enu_coordinate.y() += adc_position_.y();
    //     pose->mutable_position()->set_x(enu_coordinate.x());
    //     pose->mutable_position()->set_y(enu_coordinate.y());
    // }

    Eigen::Quaternion<double> cur_orientation =
           common::math::HeadingToQuaternion<double>(cur_theta);

    pose->mutable_orientation()->set_qw(cur_orientation.w());
    pose->mutable_orientation()->set_qx(cur_orientation.x());
    pose->mutable_orientation()->set_qy(cur_orientation.y());
    pose->mutable_orientation()->set_qz(cur_orientation.z());

    pose->set_heading(cur_theta);

    // Set linear_velocity
    pose->mutable_linear_velocity()->set_x(std::cos(cur_theta) * state_.v_);
    pose->mutable_linear_velocity()->set_y(std::sin(cur_theta) * state_.v_);
    pose->mutable_linear_velocity()->set_z(0);

    // Set angular_velocity in both map reference frame and vehicle
    // reference
    double kappa;
    kappa = std::tan(state_.steer_) / veh_params_.wheel_base();

    pose->mutable_angular_velocity()->set_x(0);
    pose->mutable_angular_velocity()->set_y(0);
    if (state_.v_ > 0.0)
    {
        pose->mutable_angular_velocity()->set_z(state_.v_ * kappa);
    }
    else
    {
        pose->mutable_angular_velocity()->set_z(0);
    }

    TransformToVRF(pose->angular_velocity(), pose->orientation(),
                   pose->mutable_angular_velocity_vrf());

    // Set linear_acceleration in both map reference frame and vehicle
    // reference frame
    auto *linear_acceleration = pose->mutable_linear_acceleration();
    double acc = 0.0;

    if (state_.v_ > 0.0)
    {
        acc = state_.acc_;
    }
    linear_acceleration->set_x(std::cos(cur_theta) * acc);
    linear_acceleration->set_y(std::sin(cur_theta) * acc);
    linear_acceleration->set_z(0);

    TransformToVRF(pose->linear_acceleration(), pose->orientation(),
                   pose->mutable_linear_acceleration_vrf());

    common::math::EulerAnglesZXYd euler_angles(
            cur_orientation.w(), cur_orientation.x(), cur_orientation.y(),
            cur_orientation.z());
    pose->mutable_euler_angles()->set_x(euler_angles.roll());
    pose->mutable_euler_angles()->set_y(euler_angles.pitch());
    pose->mutable_euler_angles()->set_z(euler_angles.yaw());

    return;
}

int apollo_control_msg_to_virtual_chassis_control_msg(
        CanCommond *virtual_chassis_msg,
        const apollo::control::ControlCommand &control_command)
{
    apollo::common::VehicleConfig vehicle_config =
            apollo::common::VehicleConfigHelper::GetConfig();

    double steering_wheel_angle;
    steering_wheel_angle = control_command.steering_target() / 100 *
                           vehicle_config.vehicle_param().max_steer_angle();

    double front_wheel_angle;
    front_wheel_angle =
            steering_wheel_angle / vehicle_config.vehicle_param().steer_ratio();

    // set virtual cmd
    virtual_chassis_msg->steer_ = front_wheel_angle;

    virtual_chassis_msg->acc_ = control_command.acceleration();

    return 0;
}

}  // namespace apollo