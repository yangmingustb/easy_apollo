

#include "modules/viz3d/draw_map.h"
#include "modules/viz3d/draw_geometry.h"
#include "modules/viz3d/pcl_viz.h"
#include "cyber/time/time.h"

namespace apollo
{

int draw_simple_hdmap(pcl::visualization::PCLVisualizer *viz,
                      const Pose2D *base_pose)
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

    // AINFO << "hdmap lane size " << hdmap_->impl_.lane_table_.size();

    apollo::common::math::Vec2d base_pt;

    double dist;

    std::vector<apollo::hdmap::LaneInfoConstPtr> lanes;

    common::PointENU point;
    point.set_x(base_pose->pos.x);
    point.set_y(base_pose->pos.y);
    point.set_z(0);

    double radius = 100.0;
    if (hdmap_->GetLanes(point, radius, &lanes) != 0)
    {
        AERROR << "Fail to get lanes from sim_map.";
    }

    // AINFO << "roi hdmap lane size " << lanes.size();

    double sin_theta = std::sin(base_pose->theta);
    double cos_theta = std::cos(base_pose->theta);

    double start_time = apollo::cyber::Time::Now().ToSecond();

    for (const auto &lane_ptr : lanes)
    {
        const hdmap::LaneBoundary &left = lane_ptr->lane().left_boundary();
        const hdmap::LaneBoundary &right = lane_ptr->lane().right_boundary();

        // AINFO <<" len " << lane_ptr->total_length();
        // AINFO <<" id " << lane_ptr->id().DebugString();
        // AINFO <<" point size " << lane_ptr->points().size();
        draw_hdmap_lane_boudary(left, base_pose, sin_theta, cos_theta, viz);

        draw_hdmap_lane_boudary(right, base_pose, sin_theta, cos_theta, viz);
    }

    double draw_data_time = apollo::cyber::Time::Now().ToSecond();

    AINFO << " draw map time (ms): " << (draw_data_time - start_time) * 1000;

    return 0;
}

int draw_hdmap_lane_boudary(const hdmap::LaneBoundary &bound,
                            const Pose2D *base_pose, double sin_theta,
                            double cos_theta,
                            pcl::visualization::PCLVisualizer *viz)
{
    Position2D line_local_start, line_global_start;
    Position2D line_local_end, line_global_end;
    const hdmap::Curve &curve = bound.curve();

    common::Point3D start;
    common::Point3D end;

    int line_width = 2;

    pcl_color color;

    // AINFO << "curve segmen size " << curve.segment_size();

    apollo::hdmap::LaneBoundaryType_Type type = bound.boundary_type(0).types(0);

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

            cvt_pos_global_to_local_fast(&line_local_start, &line_global_start,
                                         base_pose, sin_theta, cos_theta);

            cvt_pos_global_to_local_fast(&line_local_end, &line_global_end,
                                         base_pose, sin_theta, cos_theta);

            start.set_x(line_local_start.x);
            start.set_y(line_local_start.y);
            start.set_z(0);

            end.set_x(line_local_end.x);
            end.set_y(line_local_end.y);
            end.set_z(0);

            if (type == apollo::hdmap::LaneBoundaryType_Type_SOLID_YELLOW ||
                type == apollo::hdmap::LaneBoundaryType_Type_DOUBLE_YELLOW)
            {
                draw_local_line(viz, &start, &end, pcl_colors_yellow);
            }
            else
            {
                draw_local_line(viz, &start, &end, pcl_colors_white);
            }
        }
    }

    return 0;
}

}  // namespace apollo