#include <iostream>
#include "virtual_chassis.h"
#include "modules/common/math/pose.h"

int main(int argc, const char** argv)
{
    printf("test virutal chassis\n");

    apollo::VirtualChassis chassis;

    apollo::common::VehicleParam veh_params;

    // test 1
    veh_params.set_wheel_base(1.0);
    std::cout << "l " << veh_params.wheel_base() << std::endl;

    apollo::Pose2D pose;
    pose.pos.x = 1.0;
    pose.pos.y = 0.0;
    pose.theta = M_PI / 2;
    double r = 1.0;

    double dist = 2 * M_PI / 4.0;
    apollo::Pose2D cur_pose;

    double front_wheel_angle;
    front_wheel_angle = M_PI / 4;

    cur_pose = pose;

    chassis.UpdateChassisPose(&cur_pose, front_wheel_angle, dist, &veh_params);

    std::cout << "pose, x " << cur_pose.pos.x << "y " << cur_pose.pos.y << "heading "
              << cur_pose.theta << "\n";

    // test 2
    front_wheel_angle = -M_PI / 4;

    cur_pose = pose;

    chassis.UpdateChassisPose(&cur_pose, front_wheel_angle, dist, &veh_params);

    std::cout << "pose, x " << cur_pose.pos.x << "y " << cur_pose.pos.y << "heading "
              << cur_pose.theta << "\n";

    // test 3
    front_wheel_angle = -M_PI / 4;

    cur_pose = pose;
    cur_pose.theta = -M_PI / 2.0;

    chassis.UpdateChassisPose(&cur_pose, front_wheel_angle, dist, &veh_params);

    std::cout << "pose, x " << cur_pose.pos.x << "y " << cur_pose.pos.y << "heading "
              << cur_pose.theta << "\n";

    return 0;
}