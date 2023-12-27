#pragma once
#include "opencv_viz.h"
#include "modules/common/math/pose.h"

namespace apollo
{
int viz2d_draw_prediction(
        viz2d_image *uviz,
        const std::vector<std::vector<cv::Point2d> > &obstacle_list,
        const Pose2D*base_pose, viz2d_color color_index,
        int line_width);
} // namespace apollo
