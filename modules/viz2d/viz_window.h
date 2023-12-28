
#pragma once

#include "modules/common/status/system_state.h"
#include "opencv_viz.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/routing/routing_component.h"

#include <iostream>
#include <memory>
#include "cyber/common/file.h"

#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/common/reference_line_info.h"

#include "modules/common/util/debug_mode.h"
#include "modules/viz2d/display_config.pb.h"

namespace apollo
{

void main_window2d_init(viz2d_color background_color);

void hmap_window2d_init();

int viz2d_window_read_config(std::string config_dir);

DisplayConfig* get_windows2d_config();

viz2d_image *get_main_window2d();

viz2d_image *get_hmap_window2d();

viz2d_image *get_control_window2d();

void control_viz2d_init();

int viz2d_draw_system_state(viz2d_image *viz2d, const SystemState &states,
                           const Pose2D*base_pose);

int viz2d_draw_chassis_feedback(viz2d_image *viz2d, double v, double wheel);

int viz2d_draw_control_commond_info(viz2d_image *viz2d, double acc,
                                   double wheel);

// use radian
int viz2d_draw_front_wheel_state(viz2d_image *viz2d, const Pose2D*veh_pose,
                                double wheel_base, double steering,
                                viz2d_color text_color_index);

int viz2d_draw_pause_state(viz2d_image *viz2d,
                          const apollo::RunningTimeDebug &debug);

int viz2d_draw_drive_mode(viz2d_image *viz2d,
                          const apollo::canbus::Chassis::DrivingMode &mode);

int viz2d_draw_run_mode(viz2d_image *viz2d,
                       const apollo::ApolloRunMode &mode);

int viz2d_draw_localization_time(viz2d_image *viz2d, double time_stamp);

int viz2d_draw_replay_info(viz2d_image *viz2d, int64_t ratio);

int viz2d_draw_rtk_state(viz2d_image *viz2d,
                        const localization::LocalizationEstimate &localization,
                        const Pose2D*base_pose);

}  // namespace apollo
