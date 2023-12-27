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
int viz2d_draw_local_dash_line(viz2d_image *uviz, const Position2D *start,
                              const Position2D *end,
                              viz2d_color color_index, int width);


/*
 * Draw polygon
 *
 * \param[in]  uviz         The image handle to draw on
 * \param[in]  poly         The polygon to be drawn
 * \param[in]  ref_pose     The reference pose
 * \param[in]  color_index  The color of the polygon outline
 * \return 1 or 0
 */
int viz2d_local_planner_draw_polygon(viz2d_image *uviz,
                                    const Polygon2D *poly,
                                    const Pose2D*ref_pose,
                                    viz2d_color color_index);

int viz2d_draw_line(viz2d_image *uviz, const Position2D *start,
                   const Position2D *end, const Pose2D*ref_pose,
                   viz2d_color color_index, int width);

int viz2d_draw_direction(viz2d_image *uviz, const Pose2D*start,
                        double len, const Pose2D*ref_pose,
                        viz2d_color color_index, int width);


int viz2d_draw_circle_wrapper(viz2d_image *uviz,
                             const Position2D *circle_center,
                             const Pose2D*base_pose,
                             viz2d_color color_index, int radius,
                             bool filled);


int viz2d_draw_local_line(viz2d_image *uviz, const Position2D *start,
                         const Position2D *end, viz2d_color color_index,
                         int width);

int viz2d_draw_grid(viz2d_image *uviz, double left, double right,
                   double front, double back, const Pose2D*base_pose,
                   viz2d_color color_index, int line_width);

int cv_draw_polygon(viz2d_image *uviz, const Polygon2D *poly,
                    const Pose2D*ref_pose, viz2d_color color_index,
                    int width);

int viz_draw_virtual_wall(Pose2D*wall_pose, viz2d_image *uviz,
                          const Pose2D*veh_pose, int perception_id,
                          viz2d_color wall_color);

int viz_draw_box_in_cv_frame(viz2d_image *uviz, const CvPoint *center,
                             double height, double length,
                             viz2d_color color, bool fill);

} // namespace apollo
