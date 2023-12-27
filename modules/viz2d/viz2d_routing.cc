#include "viz2d_routing.h"
#include "modules/viz2d/viz_window.h"
#include "viz2d_geometry.h"

namespace apollo
{

int viz2d_draw_route(viz2d_image *uviz,
                    const std::vector<apollo::hdmap::MapPathPoint> &ref_line,
                    const Pose2D*base_pose, viz2d_color color_index,
                    int line_width)
{
    int ret;
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar color;

    if (ref_line.size() == 0 || base_pose == nullptr || uviz == nullptr)
    {
        return -1;
    }

    Position2D global_pose;
    Position2D start, end;

    bool index_valid;

    ret = viz2d_get_color(&color, color_index);

    for (i = 0; i < ref_line.size() - 1; i++)
    {
        global_pose.x = ref_line[i].x();
        global_pose.y = ref_line[i].y();

        // AINFO << "i " << i << "x " << global_pose.x << " y " <<
        // global_pose.y;

        cvt_pos_global_to_local(&start, &global_pose, base_pose);

        global_pose.x = ref_line[i + 1].x();
        global_pose.y = ref_line[i + 1].y();

        cvt_pos_global_to_local(&end, &global_pose, base_pose);

        viz2d_draw_local_line(uviz, &start, &end, color_index, 2);
        // cvLine(uviz->image, point1, point2, color, line_width, CV_AA, 0);
    }

    return 1;
}

int viz2d_draw_route2(viz2d_image *uviz,
                     const std::vector<apollo::hdmap::Path> &route_path_list_,
                     const Pose2D*base_pose, viz2d_color color_index,
                     int line_width)
{
    int ret;
    CvPoint point1, point2;
    CvScalar color;

    Position2D global_pose;
    Position2D start, end;

    bool index_valid;

    ret = viz2d_get_color(&color, color_index);

    if (route_path_list_.size() == 0 || base_pose == nullptr || uviz == nullptr)
    {
        return -1;
    }

    double dist;

    const apollo::hdmap::MapPathPoint *history_point = nullptr;

    for (const apollo::hdmap::Path &path : route_path_list_)
    {
        // Downsample the path points for frontend display.

        const std::vector<apollo::hdmap::MapPathPoint> &points =
                path.path_points();

        history_point = nullptr;
        for (std::size_t i = 0; i < points.size(); i++)
        {
            const apollo::hdmap::MapPathPoint &point = points.at(i);

            if (history_point != nullptr)
            {
                dist = point.DistanceTo(*history_point);

                if (dist < 1.0)
                {
                    continue;
                }

                global_pose.x = history_point->x();
                global_pose.y = history_point->y();

                // AINFO << "i " << i << "x " << global_pose.x << " y " <<
                // global_pose.y;

                cvt_pos_global_to_local(&start, &global_pose, base_pose);

                global_pose.x = point.x();
                global_pose.y = point.y();

                cvt_pos_global_to_local(&end, &global_pose, base_pose);

                viz2d_draw_local_line(uviz, &start, &end, color_index, 2);
            }

            history_point = &points.at(i);
        }
    }

    return 1;
}

int viz2d_draw_route(viz2d_image *uviz,
                    const std::vector<cv::Vec2d> &ref_line,
                    const Pose2D*base_pose, viz2d_color color_index,
                    int line_width)
{
    int ret;
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar color;

    if (ref_line.size() == 0 || base_pose == nullptr || uviz == nullptr)
    {
        return -1;
    }

    Position2D local_pose, global_pose;

    bool index_valid;

    ret = viz2d_get_color(&color, color_index);

    for (i = 0; i < ref_line.size() - 1; i++)
    {
        global_pose.x = ref_line[i][0];
        global_pose.y = ref_line[i][1];

        // AINFO << "i " << i << "x " << global_pose.x << " y " <<
        // global_pose.y;

        cvt_pos_global_to_local(&local_pose, &global_pose, base_pose);
        ret = viz2d_get_index(uviz, &point1, local_pose.x, local_pose.y);

        index_valid = check_index_valid(uviz, point1.x, point1.y, 2);
        if (!index_valid)
        {
            continue;
        }

        global_pose.x = ref_line[i + 1][0];
        global_pose.y = ref_line[i + 1][1];

        cvt_pos_global_to_local(&local_pose, &global_pose, base_pose);
        ret = viz2d_get_index(uviz, &point2, local_pose.x, local_pose.y);

        index_valid = check_index_valid(uviz, point2.x, point2.y, 2);
        if (!index_valid)
        {
            continue;
        }

        cvLine(uviz->image, point1, point2, color, line_width, CV_AA, 0);
    }

    return 1;
}

}  // namespace apollo
