
#include "draw_path.h"
#include "modules/viz3d/draw_geometry.h"

namespace apollo
{


int draw_trajectory(pcl::visualization::PCLVisualizer *viz,
                    const planning::ADCTrajectory *traj,
                    pcl_color color, const Pose2D *veh_pose,
                    const Polygon2D *veh_local_polygon)
{
    int ret;
    int i, feasible_size;
    double truncated_dist;

    Pose2D base_pose;
    Pose2D global_pose;
    Pose2D local_pose;

    base_pose = *veh_pose;

    Polygon2D global_polygon;
    Polygon2D local_polygon_to_veh;
    Polygon2D traj_local_polygon;
    traj_local_polygon = *veh_local_polygon;

    // 为了美观，不需要画出整个车体，画出车宽即可
    traj_local_polygon.vertexes[0].y = 0.3;
    traj_local_polygon.vertexes[1].y = 0.3;
    traj_local_polygon.vertexes[2].y = -0.3;
    traj_local_polygon.vertexes[3].y = -0.3;

    if (traj->trajectory_point_size() == 0)
    {
        return -1;
    }

    for (i = 0; i < traj->trajectory_point_size(); i++)
    {
        global_pose.pos.x = traj->trajectory_point(i).path_point().x();
        global_pose.pos.y = traj->trajectory_point(i).path_point().y();
        global_pose.theta = traj->trajectory_point(i).path_point().theta();

        cvt_local_polygon_to_global(&global_polygon, &traj_local_polygon,
                                    &global_pose);

        cvt_global_polygon_to_local(&local_polygon_to_veh, &global_polygon,
                                    veh_pose);

        // dump_polygon(&global_polygon);

        draw_global_polygon_plane(viz, &local_polygon_to_veh, color, veh_pose);
    }

    return 0;
}

int draw_trajectory(pcl::visualization::PCLVisualizer *viz,
                    const planning::ADCTrajectory *traj,
                    pcl_color_index color_index, const Pose2D *veh_pose,
                    const Polygon2D *veh_local_polygon)
{
    pcl_color color;
    pcl_get_color(&color, color_index);

    draw_trajectory(viz, traj, color, veh_pose, veh_local_polygon);
    return 0;
}

}  // namespace apollo