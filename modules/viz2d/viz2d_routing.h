#pragma once

#include "modules/common/status/system_state.h"
#include "opencv_viz.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/routing/routing_component.h"

#include <iostream>
#include <memory>
#include "cyber/common/file.h"
#include "modules/common/util/debug_mode.h"


namespace apollo
{
int viz2d_draw_route(viz2d_image *viz2d,
                    const std::vector<cv::Vec2d> &ref_line,
                    const Pose2D*base_pose, viz2d_color color_index,
                    int line_width);

int viz2d_draw_route2(viz2d_image *viz2d,
                     const std::vector<apollo::hdmap::Path> &route_path_list_,
                     const Pose2D*base_pose, viz2d_color color_index,
                     int line_width);

int viz2d_draw_route(viz2d_image *viz2d,
                    const std::vector<apollo::hdmap::MapPathPoint> &ref_line,
                    const Pose2D*base_pose, viz2d_color color_index,
                    int line_width);
} // namespace apollo
