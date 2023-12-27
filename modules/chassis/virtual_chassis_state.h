#pragma once
#include <cmath>

namespace apollo
{


// 虚拟底盘的实现，在受到指令后，零延时响应：方向盘、油门。
struct ChassisState
{
    /* data */
    double x_;
    double y_;
    double z_;

    // -pi, pi
    double heading_;

    double v_;
    double acc_;
    // -pi, +pi, front wheel angle
    // todo: left turn is positive, keep same with apollo.
    double steer_;
};

struct CanCommond
{
    /* data */
    // front wheel, -pi, +pi. left turn is positive value
    double steer_;
    double acc_;
    double v_;
};

enum class chassis_update_type
{
    perfect_response,
    time_delay,
    time_delay_and_overshoot,
};

}