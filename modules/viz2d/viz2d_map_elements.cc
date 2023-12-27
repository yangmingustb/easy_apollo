
#include "viz2d_map_elements.h"
#include "modules/viz2d/viz_window.h"

namespace apollo
{

int apollo_polygon_to_base_polygon(const apollo::common::math::Polygon2d &poly,
                                   Polygon2D *poly_dst)
{
    const std::vector<common::math::Vec2d> &points = poly.points();

    poly_dst->vertex_num = 0;
    for (size_t i = 0; i < points.size(); i++)
    {
        poly_dst->vertexes[i].x = points.at(i).x();
        poly_dst->vertexes[i].y = points.at(i).y();

        poly_dst->vertex_num++;
    }

    return 0;
}

int viz2d_draw_apollo_polygon(viz2d_image *image_handle,
                             const apollo::common::math::Polygon2d &poly,
                             const Pose2D*base_pose,
                             viz2d_color color_index, int32_t thickness)
{
    Polygon2D polygon;
    apollo_polygon_to_base_polygon(poly, &polygon);

    viz2d_draw_polygon_with_thickness(image_handle, &polygon, base_pose,
                                     color_index, thickness);

    return 0;
}

int viz2d_draw_crosswalk(viz2d_image *uviz, const Pose2D*base_pose,
                        viz2d_color color_index, double dist_thresh,
                        int line_width)
{
    const apollo::hdmap::HDMap *hdmap_;

    hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
    // hdmap_ = apollo::hdmap::HDMapUtil::SimMapPtr();

    if (hdmap_ == nullptr)
    {
        return -1;
    }

    apollo::common::PointENU point;
    point.set_x(base_pose->pos.x);
    point.set_y(base_pose->pos.y);
    point.set_z(0);

    std::vector<hdmap::CrosswalkInfoConstPtr> crosswalks;

    if (hdmap_->GetCrosswalks(point, dist_thresh, &crosswalks) != 0)
    {

        AERROR << "fail to get cross walk";
    }

    for (size_t i = 0; i < crosswalks.size(); i++)
    {
        hdmap::CrosswalkInfoConstPtr crosswalk = crosswalks.at(i);

        if(crosswalk == nullptr)
        {
            continue;
        }

        const apollo::common::math::Polygon2d &poly = crosswalk->polygon();

        if(poly.num_points() <=0)
        {
            continue;
        }

        viz2d_draw_apollo_polygon(uviz, poly, base_pose, color_index,
                                 line_width);
    }
    

    return 0;
}


int viz2d_draw_hdmap(viz2d_image *uviz, const Pose2D*base_pose,
                    bool draw_full_map)
{
    Position2D lane_line_start, lane_line_end_pt;

    const apollo::hdmap::HDMap *hdmap_;

    hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
    // hdmap_ = apollo::hdmap::HDMapUtil::SimMapPtr();

    if (hdmap_ == nullptr)
    {
        return -1;
    }
    apollo::hdmap::LaneInfoConstPtr lane_ptr;

    double left_width, right_width;

    // AINFO << "hdmap lane size " << hdmap_->impl_.lane_table_.size();

    apollo::common::math::Vec2d base_pt;

    Pose2D start, end;
    Position2D line_local, line_global;

    double dist;

    for (const auto &lane_ptr_pair : hdmap_->impl_.lane_table_)
    {
        lane_ptr = lane_ptr_pair.second;

        const std::vector<apollo::common::math::Vec2d> &points =
                lane_ptr->points();

        lane_ptr->GetWidth(0, &left_width, &right_width);

        std::size_t pt_size = points.size();
        for (std::size_t j = 0; j < pt_size - 1; j++)
        {
            const apollo::common::math::Vec2d &pt_start = points[j];

            start.pos.x = pt_start.x();
            start.pos.y = pt_start.y();

            if (!draw_full_map)
            {
                dist = calc_point2d_dist(&start.pos, &base_pose->pos);

                if (dist > 200.0)
                {
                    continue;
                }
            }

            const apollo::common::math::Vec2d &pt_end = points[j + 1];
            end.pos.x = pt_end.x();
            end.pos.y = pt_end.y();

            start.theta = apollo_calc_theta(&start.pos, &end.pos);
            start.theta = apollo_unify_theta(start.theta, apollo_PI);
            end.theta = start.theta;

            // left line start
            line_local.x = -left_width;
            line_local.y = 0;

            cvt_pos_local_to_global(&line_global, &line_local, &start);

            lane_line_start.x = line_global.x;
            lane_line_start.y = line_global.y;

            // left line end
            line_local.x = -left_width;
            line_local.y = 0;

            cvt_pos_local_to_global(&line_global, &line_local, &end);

            lane_line_end_pt.x = line_global.x;
            lane_line_end_pt.y = line_global.y;

            viz2d_draw_line(uviz, &lane_line_start, &lane_line_end_pt, base_pose,
                           viz2d_colors_gray, 2);

            // right start
            line_local.x = right_width;
            line_local.y = 0;

            cvt_pos_local_to_global(&line_global, &line_local, &start);

            lane_line_start.x = line_global.x;
            lane_line_start.y = line_global.y;
            // right end

            line_local.x = right_width;
            line_local.y = 0;

            cvt_pos_local_to_global(&line_global, &line_local, &end);

            lane_line_end_pt.x = line_global.x;
            lane_line_end_pt.y = line_global.y;

            viz2d_draw_line(uviz, &lane_line_start, &lane_line_end_pt, base_pose,
                           viz2d_colors_gray, 2);
        }
    }

    return 0;
}

int viz2d_draw_full_simple_hdmap(viz2d_image *uviz,
                                const Pose2D*base_pose)
{
    Position2D lane_line_start, lane_line_end_pt;

    const apollo::hdmap::HDMap *hdmap_;

    // hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
    hdmap_ = apollo::hdmap::HDMapUtil::SimMapPtr();

    if (hdmap_ == nullptr)
    {
        return -1;
    }
    apollo::hdmap::LaneInfoConstPtr lane_ptr;

    double left_width, right_width;

    AINFO << "hdmap lane size " << hdmap_->impl_.lane_table_.size();

    apollo::common::math::Vec2d base_pt;

    Pose2D start, end;
    Position2D line_local, line_global;

    double dist;

    double sin_theta = std::sin(base_pose->theta);
    double cos_theta = std::cos(base_pose->theta);

    for (const auto &lane_ptr_pair : hdmap_->impl_.lane_table_)
    {
        lane_ptr = lane_ptr_pair.second;

        const hdmap::LaneBoundary &left = lane_ptr->lane().left_boundary();
        const hdmap::LaneBoundary &right = lane_ptr->lane().right_boundary();

        viz2d_draw_hdmap_lane_boudary(left, base_pose, sin_theta, cos_theta,
                                     uviz);
        viz2d_draw_hdmap_lane_boudary(right, base_pose, sin_theta, cos_theta,
                                     uviz);
    }

    return 0;
}

int viz2d_draw_hdmap_lane_boudary(const hdmap::LaneBoundary &bound,
                                 const Pose2D*base_pose, double sin_theta,
                                 double cos_theta, viz2d_image *uviz)
{
    Position2D line_local_start, line_global_start;
    Position2D line_local_end, line_global_end;
    const hdmap::Curve &curve = bound.curve();

    int line_width = 2;

    viz2d_color color;

    // AINFO << "curve segmen size " << curve.segment_size();

    apollo::hdmap::LaneBoundaryType_Type type = bound.boundary_type(0).types(0);

    // 虚拟车道,尽量画密集一些
    if (bound.has_virtual_() && bound.virtual_())
    {
        for (size_t i = 0; i < curve.segment_size(); i++)
        {
            const hdmap::CurveSegment &seg = curve.segment(i);

            const hdmap::LineSegment &line_segment = seg.line_segment();

            // AINFO << "line_segment point size " << line_segment.point_size();

            for (size_t j = 0; j < line_segment.point_size() - 1; j++)
            {
                line_global_start.x = line_segment.point(j).x();
                line_global_start.y = line_segment.point(j).y();
                line_global_end.x = line_segment.point(j + 1).x();
                line_global_end.y = line_segment.point(j + 1).y();

                cvt_pos_global_to_local_fast(&line_local_start,
                                             &line_global_start, base_pose,
                                             sin_theta, cos_theta);
                cvt_pos_global_to_local_fast(&line_local_end, &line_global_end,
                                             base_pose, sin_theta, cos_theta);

                if (type == apollo::hdmap::LaneBoundaryType_Type_SOLID_WHITE)
                {
                    viz2d_draw_local_line(uviz, &line_local_start,
                                         &line_local_end, viz2d_colors_gray,
                                         line_width);
                }
                else if (type ==
                         apollo::hdmap::LaneBoundaryType_Type_SOLID_YELLOW)
                {
                    viz2d_draw_local_line(uviz, &line_local_start,
                                         &line_local_end, viz2d_colors_orange,
                                         line_width);
                }
                else if (type ==
                         apollo::hdmap::LaneBoundaryType_Type_DOTTED_WHITE)
                {
                    viz2d_draw_local_dash_line(uviz, &line_local_start,
                                              &line_local_end, viz2d_colors_gray,
                                              line_width);
                }
                else
                {
                    viz2d_draw_local_dash_line(uviz, &line_local_start,
                                              &line_local_end,
                                              viz2d_colors_orange, line_width);
                }
            }
        }

        return 0;
    }

    if (type == apollo::hdmap::LaneBoundaryType_Type_SOLID_WHITE ||
        type == apollo::hdmap::LaneBoundaryType_Type_SOLID_YELLOW ||
        type == apollo::hdmap::LaneBoundaryType_Type_CURB ||
        type == apollo::hdmap::LaneBoundaryType_Type_DOUBLE_YELLOW ||
        type == apollo::hdmap::LaneBoundaryType_Type_UNKNOWN)
    {
        for (size_t i = 0; i < curve.segment_size(); i++)
        {
            const hdmap::CurveSegment &seg = curve.segment(i);

            const hdmap::LineSegment &line_segment = seg.line_segment();

            // AINFO << "line_segment point size " << line_segment.point_size();

            for (size_t j = 0; j < line_segment.point_size() - 1; j++)
            {
                line_global_start.x = line_segment.point(j).x();
                line_global_start.y = line_segment.point(j).y();
                line_global_end.x = line_segment.point(j + 1).x();
                line_global_end.y = line_segment.point(j + 1).y();

                cvt_pos_global_to_local_fast(&line_local_start,
                                             &line_global_start, base_pose,
                                             sin_theta, cos_theta);
                cvt_pos_global_to_local_fast(&line_local_end, &line_global_end,
                                             base_pose, sin_theta, cos_theta);

                if (type == apollo::hdmap::LaneBoundaryType_Type_SOLID_YELLOW ||
                    type == apollo::hdmap::LaneBoundaryType_Type_DOUBLE_YELLOW)
                {
                    viz2d_draw_local_line(uviz, &line_local_start,
                                         &line_local_end, viz2d_colors_orange,
                                         line_width);
                }
                else
                {
                    viz2d_draw_local_line(uviz, &line_local_start,
                                         &line_local_end, viz2d_colors_gray,
                                         line_width);
                }
            }
        }
    }
    else
    {
        // 6米一个点
        std::vector<Position2D> boundary_point;
        double dist;

        for (size_t i = 0; i < curve.segment_size(); i++)
        {
            const hdmap::CurveSegment &seg = curve.segment(i);

            const hdmap::LineSegment &line_segment = seg.line_segment();

            // AINFO << "line_segment point size " << line_segment.point_size();

            for (size_t j = 0; j < line_segment.point_size() - 1; j++)
            {
                line_global_start.x = line_segment.point(j).x();
                line_global_start.y = line_segment.point(j).y();
                line_global_end.x = line_segment.point(j + 1).x();
                line_global_end.y = line_segment.point(j + 1).y();

                cvt_pos_global_to_local_fast(&line_local_start,
                                             &line_global_start, base_pose,
                                             sin_theta, cos_theta);
                cvt_pos_global_to_local_fast(&line_local_end, &line_global_end,
                                             base_pose, sin_theta, cos_theta);

                if (boundary_point.size() < 1)
                {
                    boundary_point.push_back(line_local_start);
                }

                // 每隔6米，添加一个点
                dist = calc_point2d_dist(&boundary_point.back(), &line_local_end);

                int n = std::floor(dist / 6.0);
                if (n <= 0)
                {
                    continue;
                }

                apollo::common::math::Vec2d dir(
                        line_local_end.x - line_local_start.x,
                        line_local_end.y - line_local_start.y);

                dir.Normalize();

                Position2D cur;

                // 列表最后点，和线段第一个点的距离
                double first_point_dist = calc_point2d_dist(&boundary_point.back(),
                                                        &line_local_start);

                for (size_t i = 1; i <= n; i++)
                {
                    cur.x = line_local_start.x +
                            (6 * i - first_point_dist) * dir.x();
                    cur.y = line_local_start.y +
                            (6 * i - first_point_dist) * dir.y();

                    boundary_point.push_back(cur);
                }
            }
        }

        Position2D *cv_start;
        Position2D *cv_end;
        for (size_t i = 0; i < boundary_point.size() - 1; i++)
        {
            cv_start = &boundary_point.at(i);
            cv_end = &boundary_point.at(i + 1);
            if (type == apollo::hdmap::LaneBoundaryType_Type_DOTTED_WHITE)
            {
                viz2d_draw_local_dash_line(uviz, cv_start, cv_end,
                                          viz2d_colors_gray, line_width);
            }
            else if (type == apollo::hdmap::LaneBoundaryType_Type_DOTTED_YELLOW)
            {
                viz2d_draw_local_dash_line(uviz, cv_start, cv_end,
                                          viz2d_colors_orange, line_width);
            }
        }
    }

    return 0;
}

int viz2d_draw_simple_hdmap(viz2d_image *uviz, const Pose2D*base_pose)
{
    Position2D lane_line_start, lane_line_end_pt;

    const apollo::hdmap::HDMap *hdmap_;

    hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
    // hdmap_ = apollo::hdmap::HDMapUtil::SimMapPtr();

    if (hdmap_ == nullptr)
    {
        return -1;
    }
    // apollo::hdmap::LaneInfoConstPtr lane_ptr;

    double left_width, right_width;

    AINFO << "hdmap lane size " << hdmap_->impl_.lane_table_.size();

    apollo::common::math::Vec2d base_pt;

    double dist;

    std::vector<apollo::hdmap::LaneInfoConstPtr> lanes;

    common::PointENU point;
    point.set_x(base_pose->pos.x);
    point.set_y(base_pose->pos.y);
    point.set_z(0);

    double radius = 150.0;
    if (hdmap_->GetLanes(point, radius, &lanes) != 0)
    {
        AERROR << "Fail to get lanes from sim_map.";
    }

    AINFO << "roi hdmap lane size " << lanes.size();

    double sin_theta = std::sin(base_pose->theta);
    double cos_theta = std::cos(base_pose->theta);

    for (const auto &lane_ptr : lanes)
    {
        const hdmap::LaneBoundary &left = lane_ptr->lane().left_boundary();
        const hdmap::LaneBoundary &right = lane_ptr->lane().right_boundary();

        // AINFO <<" len " << lane_ptr->total_length();
        // AINFO <<" id " << lane_ptr->id().DebugString();
        // AINFO <<" point size " << lane_ptr->points().size();
        viz2d_draw_hdmap_lane_boudary(left, base_pose, sin_theta, cos_theta,
                                     uviz);
        viz2d_draw_hdmap_lane_boudary(right, base_pose, sin_theta, cos_theta,
                                     uviz);
    }

    return 0;
}


int get_hdmap_base_for_uviz(Pose2D*base_pose)
{
    const apollo::hdmap::HDMap *hdmap_;

    hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();

    if (hdmap_ == nullptr)
    {
        AERROR << "fail to read hdmap";
        return -1;
    }
    apollo::hdmap::LaneInfoConstPtr lane_ptr;

    apollo::common::math::Vec2d base_pt;

    double max_x = 0.0;
    double max_y = 0.0;
    double min_x = 0.0;
    double min_y = 0.0;
    bool init_status = false;

    for (const auto &lane_ptr_pair : hdmap_->impl_.lane_table_)
    {
        lane_ptr = lane_ptr_pair.second;

        const std::vector<apollo::common::math::Vec2d> &points =
                lane_ptr->points();

        std::size_t pt_size = points.size();
        for (std::size_t j = 0; j < pt_size; j++)
        {
            const apollo::common::math::Vec2d &pt_start = points[j];
            if (!init_status)
            {
                max_x = pt_start.x();
                min_x = pt_start.x();
                max_y = pt_start.y();
                min_y = pt_start.y();

                init_status = true;
            }
            else
            {
                max_x = std::max(max_x, pt_start.x());
                min_x = std::min(min_x, pt_start.x());

                max_y = std::max(max_y, pt_start.y());
                min_y = std::min(min_y, pt_start.y());
            }
        }
    }

    base_pose->theta = 0.0;
    base_pose->pos.x = (max_x + min_x) / 2.0;
    base_pose->pos.y = (max_y + min_y) / 2.0;

    AINFO << "hdmap center position: " << base_pose->pos.x << ", "
          << base_pose->pos.y;

    return 0;
}

}