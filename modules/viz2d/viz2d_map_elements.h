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

#include "viz2d_geometry.h"

namespace apollo
{
int viz2d_draw_crosswalk(viz2d_image *uviz, const Pose2D*base_pose,
                        viz2d_color color_index, double dist_thresh,
                        int line_width);

int apollo_polygon_to_base_polygon(const apollo::common::math::Polygon2d &poly,
                                   Polygon2D *poly_dst);

int viz2d_draw_apollo_polygon(viz2d_image *image_handle,
                             const apollo::common::math::Polygon2d &poly,
                             const Pose2D*base_pose,
                             viz2d_color color_index, int32_t thickness);

int viz2d_draw_hdmap(viz2d_image *uviz, const Pose2D*base_pose,
                    bool draw_full_map);

int viz2d_draw_simple_hdmap(viz2d_image *uviz, const Pose2D*base_pose);

int viz2d_draw_hdmap_lane_boudary(const hdmap::LaneBoundary &bound,
                                 const Pose2D*base_pose, double sin_theta,
                                 double cos_theta, viz2d_image *uviz);

int viz2d_draw_full_simple_hdmap(viz2d_image *uviz,
                                const Pose2D*base_pose);

int get_hdmap_base_for_uviz(Pose2D *base_pose);
}