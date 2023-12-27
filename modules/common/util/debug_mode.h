#pragma once

namespace apollo
{
struct PauseDebugMode
{
    // 按p暂停，再按p,取消暂停
    // 如果系统处于暂停状态，控制器不更新，规划不更新，底盘不更新
    bool enabled;
};

struct SingleFrameDebugMode
{
    // 按z，进入单帧调试模式，再按z，推出模式。
    bool enabled;

    // 在单帧调试模式中，如果没有接收到c命令，程序只会跑一帧，然后停下。
    bool receive_continue_cmd;
};

enum DebugModeEnum
{
    DebugModePause,
    DebugModeSingleFrame,
    DebugModeMaxNumber
};

// 程序运行时的debug方式，可以是暂停模式，也可是单帧模式。只能选择一个模式。
struct RunningTimeDebug
{
    DebugModeEnum debug_type;
    PauseDebugMode pause_debug;
    SingleFrameDebugMode single_frame_debug;
};

int running_time_debug_info();

int set_debug_info(const RunningTimeDebug &debug);

const RunningTimeDebug *get_debug_info();

}  // namespace apollo