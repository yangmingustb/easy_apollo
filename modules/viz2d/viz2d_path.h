#pragma once

#include "opencv_viz.h"
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/reference_line_info.h"

#include "modules/planning/proto/planning.pb.h"

namespace apollo
{

int draw_ref_line_path_boundarys(
        viz2d_image *uviz,
        const apollo::planning::ReferenceLineInfo &ref_line_info,
        const Pose2D*base_pose);

int draw_path_boundary(const planning::ReferenceLine &ref_line,
                       const planning::PathBoundary &bound,
                       viz2d_image *uviz, const Pose2D*base_pose,
                       viz2d_color color_index);

int viz2d_draw_path(viz2d_image *uviz,
                   const apollo::planning::DiscretizedPath &lateral_path,
                   const Pose2D*base_pose, viz2d_color color_index,
                   int line_width);

int viz2d_draw_ref_line_info(
        viz2d_image *uviz,
        const std::list<apollo::planning::ReferenceLineInfo> &ref_lines,
        const Pose2D*base_pose);

int init_path_flag();

int draw_path_flag(viz2d_color color_index, viz2d_image *uviz,
                   const char *str);

/*
 * Draw trajectory in the ego frame
 *
 * \param[in]  uviz         The image handle to draw on
 * \param[in]  traj         The trajectory in the ego frame
 * \param[in]  color_index  The color of the trajectory
 * \return 1 or 0
 */
int cv_draw_trajectory(viz2d_image *uviz,
                       const planning::ADCTrajectory *traj,
                       viz2d_color color_index, const Pose2D*veh_pose,
                       bool is_global_pose);

/*
 * Draw trajectory in the ego frame
 *
 * \param[in]  uviz         The image handle to draw on
 * \param[in]  traj         The trajectory in the ego frame
 * \param[in]  color_index  The color of the trajectory
 * \return 1 or 0
 */
int viz2d_local_planner_draw_trajectory(
        viz2d_image *uviz, const std::vector<cv::Vec<double, 10> > &traj,
        const Pose2D*base_pose, viz2d_color color_index,
        const Polygon2D *veh_local_polygon);

int viz2d_draw_ref_line(viz2d_image *uviz,
                       const std::list<planning::ReferenceLine> *ref_lines,
                       const Pose2D*base_pose, viz2d_color color_index);

int viz2d_draw_ref_line_info(
        viz2d_image *uviz,
        const apollo::planning::ReferenceLineInfo *ref_line_info,
        const Pose2D*base_pose, viz2d_color color_index);

}  // namespace apollo