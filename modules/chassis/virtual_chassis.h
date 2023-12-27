#pragma once

#include <cmath>
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

#include "modules/common/math/quaternion.h"

#include "virtual_chassis_state.h"

#include "modules/localization/proto/localization.pb.h"
#include "modules/common/math/pose.h"

/**
 *
 */
namespace apollo
{
#define debug_virtual_chassis 0


class VirtualChassis
{
private:
    /* data */
public:
    VirtualChassis(/* args */);
    ~VirtualChassis();

    int Init(const apollo::common::VehicleParam &veh_param)
    {
        cmd_interval_ = 0.01;
        cmd_.acc_ = 0.0;
        cmd_.v_ = 0.0;
        cmd_.steer_ = 0.0;

        SetVehicleParam(veh_param);

        return 0;
    }

    int SetCommondTimeInterval(double interval_second)
    {
        cmd_interval_ = interval_second;
        return 0;
    }

    int SetVehicleParam(const apollo::common::VehicleParam &veh_param)
    {
        veh_params_ = veh_param;

        return 0;
    }

    int SetInitChassisState(double x, double y, double z, double heading)
    {
        state_.x_ = x;
        state_.y_ = y;
        state_.z_ = z;

        state_.heading_ = heading;

        state_.v_ = 0.0;
        state_.acc_ = 0.0;
        state_.steer_ = 0.0;

        return 0;
    }

    int SetInput(const CanCommond &cmd)
    {
        cmd_ = cmd;

        return 0;
    }

    int resetCommond()
    {
        cmd_.v_ = 0;
        cmd_.acc_ = 0;
        cmd_.steer_ = 0;
        return 0;
    }

    int Process();

    int getState(ChassisState *state)
    {
        *state = state_;

        return 0;
    }

    int PerfectResponse();

    int UpdateChassisPose(Pose2D *veh_pose, double steering_angle,
                          double move_dist,
                          apollo::common::VehicleParam *veh_params);

    double calc_turn_radius(double lfr, double steering_angle);

    void TransformToVRF(const apollo::common::Point3D &point_mrf,
                        const apollo::common::Quaternion &orientation,
                        apollo::common::Point3D *point_vrf)
    {
        Eigen::Vector3d v_mrf(point_mrf.x(), point_mrf.y(), point_mrf.z());
        auto v_vrf = common::math::InverseQuaternionRotate(orientation, v_mrf);
        point_vrf->set_x(v_vrf.x());
        point_vrf->set_y(v_vrf.y());
        point_vrf->set_z(v_vrf.z());
    }

    void PublishLocalization(
            std::shared_ptr<localization::LocalizationEstimate> &localization,
            double curr_time_stamp);

    int PublishChassis(double curr_time_stamp,
                       std::shared_ptr<apollo::canbus::Chassis> &ptr_chassis);

    int chassis_response_by_action_delay();
    int chassis_response_by_action_delay_and_overshoot();

public:
    // input

    CanCommond cmd_;

    // state
    ChassisState state_;

    // time: second
    double history_time;
    double current_time;
    double cmd_interval_;

    apollo::common::VehicleParam veh_params_;

    // history
    ChassisState history_state_;
};

int apollo_control_msg_to_virtual_chassis_control_msg(
        CanCommond *virtual_chassis_msg,
        const apollo::control::ControlCommand &control_command);

}  // namespace apollo