#pragma once
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/reference_line_info.h"

#include "modules/planning/proto/planning.pb.h"

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pcl_viz.h"
#include "modules/common/math/polygon_base.h"
#include "modules/common/proto/geometry.pb.h"

namespace apollo
{

int draw_global_polygon_frame(pcl::visualization::PCLVisualizer *viz,
                              Polygon2D *global_poly,
                              pcl_color_index color_index,
                              const Pose2D *veh_pose);

int draw_global_polygon_frame(pcl::visualization::PCLVisualizer *viz,
                              Polygon2D *global_poly,
                              pcl_color color,
                              const Pose2D *veh_pose);

int draw_coordinate_system(pcl::visualization::PCLVisualizer *viz,
                           pcl_color_index color_index, const Pose2D *pose);

int draw_global_polygon_plane(pcl::visualization::PCLVisualizer *viz,
                              Polygon2D *global_poly, pcl_color color,
                              const Pose2D *veh_pose);

int draw_local_polygon3d_frame(pcl::visualization::PCLVisualizer *viz,
                              Polygon2D *poly,
                              double height,
                              pcl_color_index color_index,
                              const Pose2D *veh_pose);

int draw_local_polygon2d_frame(pcl::visualization::PCLVisualizer *viz,
                              Polygon2D *poly,
                              pcl_color_index color_index,
                              const Pose2D *veh_pose);

int draw_local_grid(pcl::visualization::PCLVisualizer *viz, double left,
                    double right, double front, double back,
                    const Pose2D *base_pose, pcl_color_index color_index,
                    int line_width);

int draw_local_line(pcl::visualization::PCLVisualizer *viz, common::Point3D *start,
                    common::Point3D *end, pcl_color_index color_index);
}