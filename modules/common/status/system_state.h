#pragma once

#include "modules/canbus/proto/chassis.pb.h"

namespace apollo
{
#define default_lon_acc_when_stopped (-1)

enum ModuleStateCode
{
    ModuleStateCodeNone = 0,
    ModuleStateCodeError = 1,
    ModuleStateCodeSuccess = 2,
    ModuleStateCodeTimeOut = 3,
    ModuleStateCodeNumber = 4,
};

enum class ModuleName
{
    chassis = 0,
    localization,
    can_cmd,
    planner,
    control,
    perception,
    routing,
    reference_line,
    communication_middle_ware,
    prediction,
    max_module_number = 10
};

#define MODULE_MAX_NUMBER (10)

enum class ApolloRunMode
{
    simple_simulation,
    reality,
    data_replay,
    carla_simulation,
    run_mode_max_num
};

enum class ApolloVisualizerType
{
    ros_viz,
    opencv_simple,
    opencv_multi_window
};

struct SystemState
{
    /* data */
    ModuleStateCode moduel_state[MODULE_MAX_NUMBER];

    ApolloRunMode run_mode;
};

int SystemStateSetSuccess(ModuleStateCode &state);

int SystemStateSetError(ModuleStateCode &state);

const char *get_module_name(ModuleName name);

int SystemStateSet(SystemState &state, ModuleName name, ModuleStateCode code);

int SystemStateReset(SystemState &state);

int SystemStateInit(SystemState &state);

int set_system_control_state(SystemState &state,
                             apollo::canbus::Chassis::DrivingMode mode);

bool IsModuleSuccess(SystemState &state, ModuleName name);

int SystemStateFinish();

const char *get_run_mode_name(ApolloRunMode name);

}  // namespace apollo