#include "system_state.h"

#include <iostream>


namespace apollo
{
int SystemStateSetSuccess(ModuleStateCode &state)
{
    state = ModuleStateCodeSuccess;
    return 0;
}

int SystemStateSet(SystemState &state, ModuleName name, ModuleStateCode code)
{
    state.moduel_state[static_cast<int>(name)] = code;
    return 0;
}

int SystemStateSetError(ModuleStateCode &state)
{
    state = ModuleStateCodeError;
    return 0;
}

static char run_mode_name[static_cast<int>(ApolloRunMode::run_mode_max_num)]
                         [64] = {"simulation", "reality", "replay",
                                 "carla simulation"};

const char *get_run_mode_name(ApolloRunMode name)
{
    return (name < (ApolloRunMode::run_mode_max_num))
                   ? run_mode_name[static_cast<int>(name)]
                   : "Invalid Error";
}

static char module_name[static_cast<int>(ModuleName::max_module_number)][64] = {
        "chassis",   "localization",   "can_cmd",
        "planner",   "control",        "perception",
        "routing",   "reference_line", "node communication",
        "prediction"};

const char *get_module_name(ModuleName name)
{
    return (name < ModuleName::max_module_number)
                   ? module_name[static_cast<int>(name)]
                   : "Invalid Error";
}

int SystemStateReset(SystemState &state)
{
    for (int i = 0; i < static_cast<int>(ModuleName::max_module_number); i++)
    {
        state.moduel_state[i] = ModuleStateCodeNone;
    }

    return 0;
}

int SystemStateInit(SystemState &state)
{
    for (int i = 0; i < static_cast<int>(ModuleName::max_module_number); i++)
    {
        state.moduel_state[i] = ModuleStateCodeNone;
    }

    state.run_mode = apollo::ApolloRunMode::simple_simulation;

    return 0;
}

int set_system_control_state(SystemState &state,
                             apollo::canbus::Chassis::DrivingMode mode)
{
    return 0;
}

bool IsModuleSuccess(SystemState &state, ModuleName name)
{
    if (state.moduel_state[static_cast<int>(name)] == ModuleStateCodeSuccess)
    {
        return true;
    }
    if (state.moduel_state[static_cast<int>(name)] == ModuleStateCodeTimeOut)
    {
        return true;
    }
    return false;
}

int SystemStateFinish()
{
    // StopLogger();
    return 0;
}

}  // namespace apollo