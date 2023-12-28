#include "viz2d_geometry.h"

namespace apollo
{
    
int viz2d_draw_local_dash_line(viz2d_image *viz2d, const Position2D *start,
                              const Position2D *end,
                              viz2d_color color_index, int width)
{
    int i;
    CvPoint pt1, pt2;
    CvScalar color;
    int ret;
    Position2D next;

    ret = viz2d_get_color(&color, color_index);

    // 虚线段长度6米
    // 虚线间隔6米
    double dist = calc_point2d_dist(start, end);

    // 2个端点距离小，画一半
    if (dist <= 12.0)
    {
        next.x = (start->x + end->x) / 2;
        next.y = (start->y + end->y) / 2;

        viz2d_get_index(viz2d, &pt1, start->x, start->y);
        viz2d_get_index(viz2d, &pt2, next.x, next.y);
        cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);
    }
    else
    {
        int n = std::floor(dist / 12.0);

        apollo::common::math::Vec2d dir(end->x - start->x, end->y - start->y);
        dir.Normalize();

        Position2D cur;
        for (size_t i = 0; i < n; i++)
        {
            cur.x = start->x + 12 * i * dir.x();
            cur.y = start->y + 12 * i * dir.y();

            next.x = start->x + (12 * i + 6) * dir.x();
            next.y = start->y + (12 * i + 6) * dir.y();

            viz2d_get_index(viz2d, &pt1, start->x, start->y);
            viz2d_get_index(viz2d, &pt2, next.x, next.y);
            cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);
        }
    }

    return 0;
}



int viz2d_draw_polygon(viz2d_image *viz2d,
                                    const Polygon2D *poly,
                                    const Pose2D*ref_pose,
                                    viz2d_color color_index)
{
    int i;
    CvPoint pt1, pt2;
    CvScalar color;
    Position2D poly_pts[MAX_POLYGON_VERTEX_NUM];
    Position2D poly_pts_local[MAX_POLYGON_VERTEX_NUM];
    int ret;


    if (poly->vertex_num < 2) return 1;

    ret = viz2d_get_color(&color, color_index);

    for (i = 0; i < poly->vertex_num; i++)
    {
        poly_pts[i].x = poly->vertexes[i].x;
        poly_pts[i].y = poly->vertexes[i].y;

        cvt_pos_global_to_local(&poly_pts_local[i], &poly_pts[i], ref_pose);
    }

    for (i = 0; i < poly->vertex_num; i++)
    {
        viz2d_get_index(viz2d, &pt1, poly_pts_local[i].x, poly_pts_local[i].y);
        viz2d_get_index(
                viz2d, &pt2,
                poly_pts_local[(i == poly->vertex_num - 1) ? 0 : i + 1].x,
                poly_pts_local[(i == poly->vertex_num - 1) ? 0 : i + 1].y);
        cvLine(viz2d->image, pt1, pt2, color, 1, CV_AA, 0);
    }

    return 1;
}

int cv_draw_polygon(viz2d_image *viz2d,
                                    const Polygon2D *poly,
                                    const Pose2D*ref_pose,
                                    viz2d_color color_index,
                                    int width)
{
    int i;
    CvPoint pt1, pt2;
    CvScalar color;
    Position2D poly_pts[MAX_POLYGON_VERTEX_NUM];
    Position2D poly_pts_local[MAX_POLYGON_VERTEX_NUM];
    int ret;


    if (poly->vertex_num < 2) return 1;

    ret = viz2d_get_color(&color, color_index);

    for (i = 0; i < poly->vertex_num; i++)
    {
        poly_pts[i].x = poly->vertexes[i].x;
        poly_pts[i].y = poly->vertexes[i].y;

        cvt_pos_global_to_local(&poly_pts_local[i], &poly_pts[i], ref_pose);
    }

    for (i = 0; i < poly->vertex_num; i++)
    {
        viz2d_get_index(viz2d, &pt1, poly_pts_local[i].x, poly_pts_local[i].y);
        viz2d_get_index(
                viz2d, &pt2,
                poly_pts_local[(i == poly->vertex_num - 1) ? 0 : i + 1].x,
                poly_pts_local[(i == poly->vertex_num - 1) ? 0 : i + 1].y);
        cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);
    }

    return 1;
}

int viz2d_draw_line(viz2d_image *viz2d, const Position2D *start,
                   const Position2D *end, const Pose2D*ref_pose,
                   viz2d_color color_index, int width)
{
    int i;
    CvPoint pt1, pt2;
    CvScalar color;
    Position2D poly_pts_local[2];
    int ret;

    ret = viz2d_get_color(&color, color_index);

    cvt_pos_global_to_local(&poly_pts_local[0], start, ref_pose);
    cvt_pos_global_to_local(&poly_pts_local[1], end, ref_pose);

    viz2d_get_index(viz2d, &pt1, poly_pts_local[0].x, poly_pts_local[0].y);
    viz2d_get_index(viz2d, &pt2, poly_pts_local[1].x, poly_pts_local[1].y);
    cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);

    return 0;
}

int viz2d_draw_local_line(viz2d_image *viz2d, const Position2D *start,
                         const Position2D *end, viz2d_color color_index,
                         int width)
{
    int i;
    CvPoint pt1, pt2;
    CvScalar color;
    int ret;

    ret = viz2d_get_color(&color, color_index);

    double dist;
    dist = calc_point2d_dist(end, start);

    if (dist < 6)
    {
        viz2d_get_index(viz2d, &pt1, start->x, start->y);
        viz2d_get_index(viz2d, &pt2, end->x, end->y);
        cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);
        return 0;
    }

    // 2米插值
    int n = std::floor(dist / 2.0);
    if (n <= 0)
    {
        return 0;
    }

    apollo::common::math::Vec2d dir(end->x - start->x, end->y - start->y);
    dir.Normalize();

    Position2D cur;
    cur = *start;
    Position2D next;

    double accumalte_dist = 0;

    for (size_t i = 1; i <= n; i++)
    {
        next.x = cur.x + 2 * dir.x();
        next.y = cur.y + 2 * dir.y();

        viz2d_get_index(viz2d, &pt1, cur.x, cur.y);
        viz2d_get_index(viz2d, &pt2, next.x, next.y);
        cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);

        cur = next;
        accumalte_dist += 2.0;
    }

    if (accumalte_dist < dist)
    {
        viz2d_get_index(viz2d, &pt1, cur.x, cur.y);
        viz2d_get_index(viz2d, &pt2, end->x, end->y);
        cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);
    }

    return 0;
}

int viz2d_draw_direction(viz2d_image *viz2d, const Pose2D*start,
                        double len, const Pose2D*ref_pose,
                        viz2d_color color_index, int width)
{
    Pose2D end;
    end.pos.x = start->pos.x + len * std::cos(start->theta);
    end.pos.y= start->pos.y + len * std::sin(start->theta);

    end.theta = start->theta;

    viz2d_draw_line(viz2d, &start->pos, &end.pos, ref_pose, color_index, width);

    // left
    Position2D local;
    Position2D global;

    local.x = 1.0;
    local.y = -1.0;

    cvt_pos_local_to_global(&global, &local, &end);

    viz2d_draw_line(viz2d, &end.pos, &global, ref_pose, color_index, width);

    //right

    local.x = -1.0;
    local.y = -1.0;

    cvt_pos_local_to_global(&global, &local, &end);

    viz2d_draw_line(viz2d, &end.pos, &global, ref_pose, color_index, width);

    return 0;
}

int viz2d_draw_circle_wrapper(viz2d_image *viz2d,
                             const Position2D *circle_center,
                             const Pose2D*base_pose,
                             viz2d_color color_index, int radius,
                             bool filled)
{
    int ret;
    CvPoint cv_center;
    CvScalar color;

    if (circle_center == nullptr || base_pose == nullptr || viz2d == nullptr)
    {
        return -1;
    }

    Position2D local;

    ret = viz2d_get_color(&color, color_index);

    cvt_pos_global_to_local(&local, circle_center, base_pose);

    ret = viz2d_get_index(viz2d, &cv_center, local.x, local.y);

    int thickness = 1;
    if (filled)
    {
        thickness = -1;
    }
    cvCircle(viz2d->image, cv_center, radius, color, thickness, CV_AA, 0);
    return 0;
}

int viz2d_draw_grid(
        viz2d_image *viz2d,
        double left ,double right, double front, double back,
        const Pose2D*base_pose, viz2d_color color_index, int line_width)
{
    int ret;
    CvPoint point1, point2;
    std::size_t i, feasible_size;
    double truncated_dist;
    CvScalar color;

    if (base_pose == nullptr || viz2d == nullptr)
    {
        return -1;
    }

    Position2D local_pose, global_pose;

    ret = viz2d_get_color(&color, color_index);

    Position2D left_point;
    left_point.x = -left;
    left_point.y = -back;

    Position2D right_point;
    right_point.x = right;
    right_point.y = -back;

    double delta_y = 10;
    double delta_x = 10;

    while (left_point.y < front)
    {
        ret = viz2d_get_index(viz2d, &point1, left_point.x, left_point.y);

        ret = viz2d_get_index(viz2d, &point2, right_point.x, right_point.y);

        cvLine(viz2d->image, point1, point2, color, line_width, CV_AA, 0);

        left_point.y += delta_y;
        right_point.y+= delta_y;
    }



    Position2D front_point;
    front_point.x = -left;
    front_point.y = front;

    Position2D back_point;
    back_point.x = -left;
    back_point.y = -back;


    while (front_point.x < right)
    {
        ret = viz2d_get_index(viz2d, &point1, front_point.x, front_point.y);

        ret = viz2d_get_index(viz2d, &point2, back_point.x, back_point.y);

        cvLine(viz2d->image, point1, point2, color, line_width, CV_AA, 0);

        front_point.x+= delta_x;
        back_point.x += delta_x;
    }

    return 1;
}

int viz_draw_virtual_wall(Pose2D*wall_pose, viz2d_image *viz2d,
                          const Pose2D*veh_pose, int perception_id,
                          viz2d_color wall_color)
{

    Polygon2D stop_wall_local;
    Polygon2D stop_wall_global;


    /**
     * @brief  
     * 
     * 
     *                              
     *                      y
     *                           |
     *                           |
     *            2              |             1
     *                           |
     *                           |
     *                           |             0
     *            3              ----------------->  x
     * @note   
     * @retval None
     */

    double wall_len = 5.0; 
    double wall_width = 0.4;

    stop_wall_local.vertexes[0].x = wall_len / 2;
    stop_wall_local.vertexes[0].y = 0;

    stop_wall_local.vertexes[1].x =  wall_len / 2;
    stop_wall_local.vertexes[1].y =  wall_width;

    stop_wall_local.vertexes[2].x =  -wall_len / 2;
    stop_wall_local.vertexes[2].y =  wall_width;

    stop_wall_local.vertexes[3].x =  -wall_len / 2;
    stop_wall_local.vertexes[3].y =  0;

    stop_wall_local.vertex_num = 4;

    cvt_local_polygon_to_global(&stop_wall_global, &stop_wall_local,
                                wall_pose);

    viz2d_draw_filled_polygon(viz2d, &stop_wall_global, wall_color,
                                           veh_pose);

    return 0;
}

int viz_draw_box_in_cv_frame(viz2d_image *viz2d, const CvPoint *center,
                             double height, double length,
                             viz2d_color color, bool fill)
{
    CvScalar line_color;
    CvScalar box_fill_color;
    CvPoint points[4];
    CvPoint point;
    uint32_t i;
    int size;
    int ret;


    /**
     * @brief  
     * @note   
     * @retval None
     * 
     * 
     * 
     *                                        x
     *              ------------------------>
     *              |
     *              |
     *              |
     *              |         -----------   1
     *              |        |          \
     *              |        |    *     \
     *              |        |--------- \   0
     *              |
     *        y     V
     */            

    point.x = center->x + length / 2;
    point.y = center->y + height / 2;
    points[0] = point;

    point.x = center->x + length / 2;
    point.y = center->y - height / 2;
    points[1] = point;

    point.x = center->x - length / 2;
    point.y = center->y - height / 2;
    points[2] = point;

    point.x = center->x - length / 2;
    point.y = center->y + height / 2;
    points[3] = point;

    viz2d_get_color(&line_color, color);

    if (fill)
    {
        viz2d_get_color(&box_fill_color, viz2d_colors_black_gray);
        cvFillConvexPoly(viz2d->image, points, 4, box_fill_color, CV_AA, 0);
    }

    for (size_t i = 0; i < 4; i++)
    {
        if (i == 3)
        {
            cvLine(viz2d->image, points[i], points[0], line_color, 1, CV_AA, 0);
        }
        else
        {
            cvLine(viz2d->image, points[i], points[i + 1], line_color, 1, CV_AA,
                   0);
        }
    }

    return 0;
}

} // namespace apollo
