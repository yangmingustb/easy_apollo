cmake_minimum_required(VERSION 3.5)

include_directories(${CMAKE_HOME_DIRECTORY}/build)

add_subdirectory(audio)
# add_subdirectory(bridge)
# add_subdirectory(data)

add_subdirectory(canbus)
add_subdirectory(common)
add_subdirectory(control)

add_subdirectory(drivers)
add_subdirectory(localization)
add_subdirectory(map)
add_subdirectory(monitor)
add_subdirectory(perception)
add_subdirectory(planning)
add_subdirectory(prediction)
add_subdirectory(routing)
add_subdirectory(storytelling)
add_subdirectory(task_manager)
add_subdirectory(transform)
add_subdirectory(v2x)
add_subdirectory(chassis)
add_subdirectory(viz2d)
add_subdirectory(common_msgs)
add_subdirectory(viz3d)

add_subdirectory(dreamview)
add_subdirectory(gui)
# add_subdirectory(tools)

add_subdirectory(dreamview_plus)