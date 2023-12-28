#include "viz2d_path.h"

namespace apollo
{

using namespace apollo::planning;

CvPoint path_flag_text_center;
int path_flag_text_center_delta_y = 20;

int init_path_flag()
{
    path_flag_text_center.x = 100;
    path_flag_text_center.y = 20;

    path_flag_text_center_delta_y = 20;
    return 0;
}

int draw_path_boundary(const ReferenceLine &ref_line,
                       const planning::PathBoundary &bound,
                       viz2d_image *viz2d, const Pose2D*base_pose,
                       viz2d_color color_index)
{
    const std::vector<std::pair<double, double>> &boundary = bound.boundary();

    double start_s = bound.start_s();
    double delta_s = bound.delta_s();

    common::SLPoint sl_left;
    common::SLPoint sl_right;

    common::math::Vec2d xy_left;
    common::math::Vec2d xy_right;

    Position2D global;
    Position2D local;

    double cos_theta = std::cos(base_pose->theta);
    double sin_theta = std::sin(base_pose->theta);

    double cur_s;

    double radius = 0.1;



    for (size_t i = 0; i < boundary.size(); i++)
    {
        cur_s = start_s + delta_s * i;

        sl_left.set_s(cur_s);
        sl_left.set_l(boundary[i].second);

        ref_line.SLToXY(sl_left, &xy_left);

        global.x = xy_left.x();
        global.y = xy_left.y();

        cvt_pos_global_to_local_fast(&local, &global, base_pose, sin_theta,
                                     cos_theta);

        viz2d_draw_circle(viz2d, local.x, local.y, radius, color_index, -1);

        sl_right.set_s(cur_s);
        sl_right.set_l(boundary[i].first);

        ref_line.SLToXY(sl_right, &xy_right);

        global.x = xy_right.x();
        global.y = xy_right.y();

        cvt_pos_global_to_local_fast(&local, &global, base_pose, sin_theta,
                                     cos_theta);

        viz2d_draw_circle(viz2d, local.x, local.y, radius, color_index, -1);
    }
    

    return 0;
}

int draw_path_flag(viz2d_color color_index, viz2d_image *viz2d,
                   const char *str)
{
    CvScalar text_color;

    CvFont windows_font;

    viz2d_get_cvfont(&windows_font, &(viz2d->font));

    // text

    viz2d_get_color(&text_color, color_index);

    cvPutText(viz2d->image, str, path_flag_text_center, &windows_font,
              text_color);

    path_flag_text_center.y += path_flag_text_center_delta_y;

    return 0;
}

int draw_ref_line_path_boundarys(
        viz2d_image *viz2d,
        const apollo::planning::ReferenceLineInfo &ref_line_info,
        const Pose2D*base_pose)
{
    const std::vector<planning::PathBoundary> &boundary =
            ref_line_info.GetCandidatePathBoundaries();

    if (boundary.size() <= 0)
    {
        return 0;
    }

    const ReferenceLine& ref_line = ref_line_info.reference_line();

    viz2d_color color_index;


    char str[128];


    for (size_t i = 0; i < boundary.size(); i++)
    {

        switch (boundary[i].type_)
        {
            case path_boundary_type::PATH_BOUND_LANE_KEEP:
                color_index = viz2d_colors_purple;
                sprintf(str, "lane_keep bound");
                break;
            case path_boundary_type::PATH_BOUND_LANE_CHANGE_LEFT:
                color_index = viz2d_colors_blue;
                sprintf(str, "lane_change left bound");
                break;
            case path_boundary_type::PATH_BOUND_LANE_CHANGE_RIGHT:
                color_index = viz2d_colors_blue;
                sprintf(str, "lane_change right bound");
                break;
            case path_boundary_type::PATH_BOUND_LANE_BORROW_LEFT:
                color_index = viz2d_colors_orange;
                sprintf(str, "lane_borrow_left bound");
                break;
            case path_boundary_type::PATH_BOUND_LANE_BORROW_RIGHT:
                color_index = viz2d_colors_pink;

                sprintf(str, "lane_borrow_right bound");
                break;
            case path_boundary_type::PATH_BOUND_FALLBACK:
                color_index = viz2d_colors_red;
                sprintf(str, "fallback bound");
                break;

            default:
                break;
        }

        draw_path_boundary(ref_line, boundary[i], viz2d, base_pose, color_index);

        draw_path_flag(color_index, viz2d, str);
    }

    const std::vector<planning::PathData> &path_list =
            ref_line_info.GetCandidatePathData();

    if (path_list.size() <= 0)
    {
        return 0;
    }

    std::string path_flag ="none";

    for (size_t i = 0; i < path_list.size(); i++)
    {
        const planning::PathData &path = path_list[i];

        switch (path.type_)
        {
            case path_boundary_type::PATH_BOUND_LANE_KEEP:
                color_index = viz2d_colors_purple;
                path_flag = "lane keep path";
                break;
            case path_boundary_type::PATH_BOUND_LANE_CHANGE_LEFT:
                color_index = viz2d_colors_blue;
                path_flag = "lane change left path";
                break;
            case path_boundary_type::PATH_BOUND_LANE_CHANGE_RIGHT:
                color_index = viz2d_colors_blue;
                path_flag = "lane change right path";
                break;
            case path_boundary_type::PATH_BOUND_LANE_BORROW_LEFT:
                color_index = viz2d_colors_orange;
                path_flag = "borrow left path";
                break;
            case path_boundary_type::PATH_BOUND_LANE_BORROW_RIGHT:
                color_index = viz2d_colors_pink;
                path_flag = "borrow right path";
                break;
            case path_boundary_type::PATH_BOUND_FALLBACK:
                color_index = viz2d_colors_red;
                path_flag = "fallback path";
                break;

            default:
                break;
        }

        viz2d_draw_path(viz2d, path.discretized_path(), base_pose, color_index,
                       1);

        draw_path_flag(color_index, viz2d, path_flag.c_str());
    }
    

    return 0;
}

// 需要debug path起点的位置
int viz2d_draw_path(viz2d_image *viz2d,
                   const apollo::planning::DiscretizedPath &lateral_path,
                   const Pose2D*base_pose, viz2d_color color_index,
                   int line_width)
{
    int ret;
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar color;
    CvScalar circle_color;

    if (base_pose == nullptr || viz2d == nullptr)
    {
        return -1;
    }
    if (lateral_path.size() <= 1)
    {
        return -1;
    }

    Position2D local_pose, global_pose;

    double cos_theta = std::cos(base_pose->theta);
    double sin_theta = std::sin(base_pose->theta);

    ret = viz2d_get_color(&color, color_index);
    ret = viz2d_get_color(&circle_color, viz2d_colors_green);

    double s = 0.0;

    for (i = 0; i < lateral_path.size() - 1; i++)
    {
        global_pose.x = lateral_path.at(i).x();
        global_pose.y = lateral_path.at(i).y();

        // AINFO << "i " << i << "x " << global_pose.x << " y " <<
        // global_pose.y;

        cvt_pos_global_to_local_fast(&local_pose, &global_pose, base_pose,
                                     sin_theta, cos_theta);
        ret = viz2d_get_index(viz2d, &point1, local_pose.x, local_pose.y);

        // draw planning start point
        s = lateral_path.at(i).s();
        if (std::fabs(s) < 0.01)
        {
            cvCircle(viz2d->image, point1, 5, circle_color, -1, CV_AA, 0);
        }

        global_pose.x = lateral_path.at(i + 1).x();
        global_pose.y = lateral_path.at(i + 1).y();

        cvt_pos_global_to_local_fast(&local_pose, &global_pose, base_pose,
                                     sin_theta, cos_theta);

        ret = viz2d_get_index(viz2d, &point2, local_pose.x, local_pose.y);

        cvLine(viz2d->image, point1, point2, color, line_width, CV_AA, 0);
    }

    return 1;
}

int draw_ref_line(viz2d_color color_index, viz2d_image *viz2d,
                  const ReferenceLine &ref_line, const Pose2D*base_pose)
{
    const ReferencePoint *start, *end;
    CvScalar color;
    viz2d_get_color(&color, color_index);

    CvPoint point1, point2;

    Position2D local, global;

    const std::vector<ReferencePoint> &ref_pts = ref_line.reference_points();

    // std::cout << "ref line pt size " << ref_pts.size() << std::endl;

    for (std::size_t i = 0; i < ref_pts.size() - 1; i++)
    {
        start = &(ref_pts.at(i));
        end = &(ref_pts.at(i + 1));

        global.x = start->x();
        global.y = start->y();

        cvt_pos_global_to_local(&local, &global, base_pose);

        viz2d_get_index(viz2d, &point1, local.x, local.y);

        // draw end
        global.x = end->x();
        global.y = end->y();

        cvt_pos_global_to_local(&local, &global, base_pose);

        viz2d_get_index(viz2d, &point2, local.x, local.y);

        cvLine(viz2d->image, point1, point2, color, 1, CV_AA, 0);
    }

    return 0;
}

int viz2d_draw_ref_line_info(
        viz2d_image *viz2d,
        const std::list<apollo::planning::ReferenceLineInfo> &ref_lines,
        const Pose2D*base_pose)
{
    int ret;

    if (base_pose == nullptr || viz2d == nullptr)
    {
        return -1;
    }

    if (ref_lines.size() == 0)
    {
        return -1;
    }

    viz2d_color color_index;

    for (const apollo::planning::ReferenceLineInfo &ref_line_info : ref_lines)
    {
        // AINFO << ref_line_info.GetCandidatePathBoundaries().size();
        // AINFO << ref_line_info.GetCandidatePathData().size();

        draw_ref_line_path_boundarys(viz2d, ref_line_info, base_pose);

        if (ref_line_info.IsDrivable())
        {
            color_index = viz2d_colors_black;
        }
        else
        {
            color_index = viz2d_colors_gray;
        }

        draw_ref_line(color_index, viz2d, ref_line_info.reference_line(),
                      base_pose);
        const std::string &str = "ref_line_" + ref_line_info.Lanes().Id();

        draw_path_flag(color_index, viz2d, str.c_str());
    }

    return 1;
}

int cv_draw_trajectory(viz2d_image *viz2d,
                       const planning::ADCTrajectory *traj,
                       viz2d_color color_index, const Pose2D*veh_pose,
                       bool is_global_pose)
{
    int ret;
    CvPoint point1, point2;
    int i, feasible_size;
    double truncated_dist;
    CvScalar color;

    Pose2D base_pose;
    Pose2D global_pose;
    Pose2D local_pose;


    viz2d_get_color(&color, color_index);

    base_pose = *veh_pose;

    for (i = 0; i < traj->trajectory_point_size() - 1; i++)
    {
        global_pose.pos.x = traj->trajectory_point(i).path_point().x();
        global_pose.pos.y = traj->trajectory_point(i).path_point().y();
        global_pose.theta = traj->trajectory_point(i).path_point().theta();

        cvt_pose_global_to_local(&local_pose, &global_pose, &base_pose);

        ret = viz2d_get_index(viz2d, &point1, local_pose.pos.x, local_pose.pos.y);

        global_pose.pos.x = traj->trajectory_point(i + 1).path_point().x();
        global_pose.pos.y = traj->trajectory_point(i + 1).path_point().y();
        global_pose.theta =
                traj->trajectory_point(i + 1).path_point().theta();

        cvt_pose_global_to_local(&local_pose, &global_pose, &base_pose);

        ret = viz2d_get_index(viz2d, &point2, local_pose.pos.x, local_pose.pos.y);

        cvLine(viz2d->image, point1, point2, color, 1, CV_AA, 0);
    }

    return 1;
}


int viz2d_draw_global_trajectory(
        viz2d_image *viz2d, const std::vector<cv::Vec<double, 10> > &traj,
        const Pose2D*base_pose, viz2d_color color_index,
        const Polygon2D *veh_local_polygon)
{
    int ret;
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar color;
    Polygon2D global_polygon;
    Polygon2D traj_local_polygon;
    traj_local_polygon = *veh_local_polygon;

    // 为了美观，不需要画出整个车体，画出车宽即可
    traj_local_polygon.vertexes[0].y = 0.05;
    traj_local_polygon.vertexes[1].y = 0.05;
    traj_local_polygon.vertexes[2].y = -0.05;
    traj_local_polygon.vertexes[3].y = -0.05;

    if (traj.size() == 0 || base_pose == nullptr || viz2d == nullptr)
    {
        return -1;
    }

    Pose2D local_pose, global_pose;

    ret = viz2d_get_color(&color, color_index);

    for (i = 0; i < traj.size() - 1; i++)
    {
        global_pose.pos.x = traj[i][3];
        global_pose.pos.y = traj[i][4];
        global_pose.theta = traj[i][5];

        cvt_local_polygon_to_global(&global_polygon, veh_local_polygon,
                                    &global_pose);

        viz2d_draw_filled_polygon(
                viz2d, &global_polygon, viz2d_colors_dodgerblue1, base_pose);
    }

    for (i = 0; i < traj.size() - 1; i++)
    {
        global_pose.pos.x = traj[i][3];
        global_pose.pos.y = traj[i][4];
        global_pose.theta = traj[i][5];

        cvt_pose_global_to_local(&local_pose, &global_pose, base_pose);
        ret = viz2d_get_index(viz2d, &point1, local_pose.pos.x, local_pose.pos.y);

        global_pose.pos.x = traj[i + 1][3];
        global_pose.pos.y = traj[i + 1][4];
        global_pose.theta = traj[i + 1][5];

        cvt_pose_global_to_local(&local_pose, &global_pose, base_pose);
        ret = viz2d_get_index(viz2d, &point2, local_pose.pos.x, local_pose.pos.y);

        cvLine(viz2d->image, point1, point2, color, 2, CV_AA, 0);
    }

    return 1;
}


int viz2d_draw_ref_line(viz2d_image *viz2d,
                       const std::list<planning::ReferenceLine> *ref_lines,
                       const Pose2D*base_pose, viz2d_color color_index)
{
    int ret;
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar color;

    if (ref_lines == nullptr || base_pose == nullptr || viz2d == nullptr)
    {
        return -1;
    }

    if (ref_lines->size() == 0)
    {
        return -1;
    }

    Position2D local, global;

    ret = viz2d_get_color(&color, color_index);

    const planning::ReferencePoint *start, *end;

    for (auto iter = ref_lines->cbegin(); iter != ref_lines->cend(); iter++)
    {
        auto &ref_line = iter;

        const std::vector<planning::ReferencePoint> &ref_pts =
                ref_line->reference_points();

        // std::cout << "ref line pt size " << ref_pts.size() << std::endl;

        for (std::size_t i = 0; i < ref_pts.size() - 1; i++)
        {
            start = &(ref_pts.at(i));
            end = &(ref_pts.at(i + 1));

            global.x = start->x();
            global.y = start->y();

            cvt_pos_global_to_local(&local, &global, base_pose);

            ret = viz2d_get_index(viz2d, &point1, local.x, local.y);

            // draw end
            global.x = end->x();
            global.y = end->y();

            cvt_pos_global_to_local(&local, &global, base_pose);

            ret = viz2d_get_index(viz2d, &point2, local.x, local.y);

            cvLine(viz2d->image, point1, point2, color, 2, CV_AA, 0);
        }
    }

    return 1;
}

int viz2d_draw_ref_line_info(
        viz2d_image *viz2d,
        const apollo::planning::ReferenceLineInfo *ref_line_info,
        const Pose2D*base_pose, viz2d_color color_index)
{
    return 0;
}

}