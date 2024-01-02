#pragma once
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/reference_line_info.h"

#include "modules/planning/proto/planning.pb.h"
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pcl_viz.h"

namespace apollo
{

int draw_trajectory(pcl::visualization::PCLVisualizer *viz,
                    const planning::ADCTrajectory *traj,
                    pcl_color color, const Pose2D *veh_pose,
                    const Polygon2D *veh_local_polygon);

int draw_trajectory(pcl::visualization::PCLVisualizer *viz,
                    const planning::ADCTrajectory *traj,
                    pcl_color_index color_index, const Pose2D *veh_pose,
                    const Polygon2D *veh_local_polygon);
}