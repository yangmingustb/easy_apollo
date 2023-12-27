#include "debug_mode.h"
#include <algorithm>
#include <mutex>

namespace apollo
{
RunningTimeDebug debug_info;

std::mutex debug_mutex;

const RunningTimeDebug *get_debug_info()
{
    std::lock_guard<std::mutex> lock(debug_mutex);

    return &debug_info;
}

int set_debug_info(const RunningTimeDebug &debug)
{
    std::lock_guard<std::mutex> lock(debug_mutex);

    debug_info = debug;
    return 0;
}

int running_time_debug_info()
{
    debug_info.pause_debug.enabled = false;

    return 0;
}
}  // namespace apollo