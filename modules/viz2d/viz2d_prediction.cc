#include "viz2d_prediction.h"

namespace apollo
{

int viz2d_draw_prediction(
        viz2d_image *viz2d,
        const std::vector<std::vector<cv::Point2d> > &obstacle_list,
        const Pose2D*base_pose, viz2d_color color_index, int line_width)
{
    int ret;
    CvPoint point1, point2;
    std::size_t i, feasible_size;
     
    CvScalar color;

    if (obstacle_list.size() == 0 || base_pose == nullptr || viz2d == nullptr)
    {
        return -1;
    }

    Position2D local_pose, global_pose;

    ret = viz2d_get_color(&color, color_index);

    for (auto &obs_traj : obstacle_list)
    {
        // draw one obs
        for (i = 0; i < obs_traj.size() - 1; i++)
        {
            // cur point
            global_pose.x = obs_traj[i].x;
            global_pose.y = obs_traj[i].y;

            cvt_pos_global_to_local(&local_pose, &global_pose, base_pose);
            ret = viz2d_get_index(viz2d, &point1, local_pose.x, local_pose.y);

            // next point
            global_pose.x = obs_traj[i + 1].x;
            global_pose.y = obs_traj[i + 1].y;

            cvt_pos_global_to_local(&local_pose, &global_pose, base_pose);
            ret = viz2d_get_index(viz2d, &point2, local_pose.x, local_pose.y);

            cvLine(viz2d->image, point1, point2, color, line_width, CV_AA, 0);
        }
    }

    return 1;
}

}  // namespace apollo
