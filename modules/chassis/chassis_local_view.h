#pragma once
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"

namespace apollo
{

struct chassis_local_view
{
    control::ControlCommand latest_control_command;
};
}  // namespace apollo