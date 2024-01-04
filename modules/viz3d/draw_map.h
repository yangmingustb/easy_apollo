#pragma once
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/reference_line_info.h"

#include "modules/planning/proto/planning.pb.h"

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pcl_viz.h"
#include "modules/common/math/polygon_base.h"

namespace apollo
{

int draw_simple_hdmap(pcl::visualization::PCLVisualizer *viz,
                      const Pose2D *base_pose);

int draw_hdmap_lane_boudary(const hdmap::LaneBoundary &bound,
                            const Pose2D *base_pose, double sin_theta,
                            double cos_theta,
                            pcl::visualization::PCLVisualizer *viz);
}