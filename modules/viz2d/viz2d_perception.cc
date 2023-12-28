
#include "viz2d_perception.h"
#include "modules/viz2d/viz_window.h"
#include "viz2d_geometry.h"

namespace apollo
{

int viz_draw_traffic_light(hdmap::SignalInfoConstPtr signal_info,
                           const Pose2D*veh_pose, viz2d_image *viz2d,
                           viz2d_color color_index, int radius)
{
    const hdmap::Signal &signal = signal_info->signal();

    Position2D center;

    int box_width = radius * 2 + 8;

    double box_width_m = box_width * viz2d->resolution;

    for (size_t i = 0; i < signal.subsignal_size(); i++)
    {
        const hdmap::Subsignal &sub_signal = signal.subsignal(i);

        center.x = sub_signal.location().x();
        center.y = sub_signal.location().y();

        viz_draw_filled_box(viz2d, &center, box_width_m, viz2d_colors_black,
                            veh_pose);

        viz2d_draw_circle_wrapper(viz2d, &center, veh_pose, color_index, radius,
                                 true);
    }

    Position2D text_center;
    CvPoint text_pos;

    CvScalar text_color;

    viz2d_get_color(&text_color, viz2d_colors_yellow);

    CvFont windows_font;

    viz2d_get_cvfont(&windows_font, &(viz2d->font));


    const hdmap::Subsignal &sub_signal = signal.subsignal(0);

    center.x = sub_signal.location().x();
    center.y = sub_signal.location().y();

    cvt_pos_global_to_local(&text_center, &center, veh_pose);

    viz2d_get_index(viz2d, &text_pos, text_center.x, text_center.y);

    cvPutText(viz2d->image, signal.id().id().c_str(), text_pos, &windows_font,
              text_color);

    return 0;
}

int viz_draw_traffic_lights(perception::TrafficLightDetection *lights,
                            const Pose2D*veh_pose,
                            viz2d_image *viz2d)
{

    if(lights == nullptr)
    {
        return 0;
    }

    if (lights->traffic_light_size() < 1)
    {
        return 0;
    }

    const auto *hdmap = hdmap::HDMapUtil::BaseMapPtr();
    if (!hdmap)
    {
        AERROR << "Invalid HD Map.";
        return 0;
    }

    viz2d_color color_index;

    int radius = 8;

    for (size_t i = 0; i < lights->traffic_light_size(); i++)
    {
        const perception::TrafficLight &light = lights->traffic_light(i);

        hdmap::SignalInfoConstPtr traffic_light_info_ptr =
                hdmap::HDMapUtil::BaseMap().GetSignalById(
                        hdmap::MakeMapId(light.id()));

        if (traffic_light_info_ptr != nullptr)
        {
            // const auto &signal = traffic_light_info_ptr->signal();

            switch (light.color())
            {
                case perception::TrafficLight::UNKNOWN:
                    color_index = viz2d_colors_black;
                    break;
                case perception::TrafficLight::RED:
                    color_index = viz2d_colors_red;
                    break;
                case perception::TrafficLight::YELLOW:
                    color_index = viz2d_colors_yellow;
                    break;
                case perception::TrafficLight::GREEN:
                    color_index = viz2d_colors_green;
                    break;
                case perception::TrafficLight::BLACK:
                    color_index = viz2d_colors_black;
                    break;
                default:
                    color_index = viz2d_colors_black;
                    break;
            }

            viz_draw_traffic_light(traffic_light_info_ptr, veh_pose, viz2d,
                                   color_index, radius);
        }
    }

    return 0;
}

int update_obs_list_pose_by_heading(perception::PerceptionObstacles *obs_list,
                                    const Pose2D&base_pose)
{

    Polygon2D obs_global;

    if (obs_list->perception_obstacle_size() <= 0)
    {
        return 0;
    }

    double curr_time = apollo::cyber::Time::Now().ToSecond();

    double v;
    double delta_time;
    double move_dist;
    double cos_theta;
    double sin_theta;
    Pose2D start_pose;
    Pose2D end_pose;

    double dist_to_ego;


    apollo::perception::PerceptionObstacle *obs;

    Polygon2D *obs_local;
    obs_local = get_obs_local_polygon();

    for (int i = 0; i < obs_list->perception_obstacle_size(); i++)
    {
        obs = obs_list->mutable_perception_obstacle(i);

        if (!obs->has_id())
        {
            continue;
        }

        start_pose.pos.x = obs->position().x();
        start_pose.pos.y = obs->position().y();

        // delete
        dist_to_ego = calc_point2d_dist(&base_pose.pos, &start_pose.pos);
        if (dist_to_ego > 200.0)
        {
            obs_list->mutable_perception_obstacle()->DeleteSubrange(i, 1);
            continue;
        }

        delta_time = curr_time - obs->timestamp();

        v = apollo_sqrt(obs->velocity().x() * obs->velocity().x() +
                     obs->velocity().y() * obs->velocity().y());
        move_dist = delta_time * v;

        // update pose by move direction
        cos_theta = std::cos(obs->theta());
        sin_theta = std::sin(obs->theta());

        end_pose.pos.x = obs->position().x() + move_dist * cos_theta;
        end_pose.pos.y = obs->position().y() + move_dist * sin_theta;

        end_pose.theta = obs->theta();

        common::Point3D *pt = obs->mutable_position();
        pt->set_x(end_pose.pos.x);
        pt->set_y(end_pose.pos.y);
        pt->set_z(0);

        // AINFO <<"obs id: "<< obs->id();
        // AINFO <<"obs theta: "<< obs->theta();

        cvt_local_polygon_to_global(&obs_global, obs_local, &end_pose);

        // polygon_point
        obs->clear_polygon_point();
        for (uint j = 0; j < obs_global.vertex_num; ++j)
        {
            common::Point3D *poly = obs->add_polygon_point();
            poly->set_x(obs_global.vertexes[j].x);
            poly->set_y(obs_global.vertexes[j].y);
        }

        // update time
        obs->set_timestamp(curr_time);
    }

    obs_list->mutable_header()->set_module_name("perception");
    obs_list->mutable_header()->set_timestamp_sec(curr_time);

    return 0;
}

int update_obs_list_pose_by_key(perception::PerceptionObstacles *obs_list,
                                bool left, bool right, bool up, bool low)
{

    Polygon2D obs_global;

    if (obs_list->perception_obstacle_size() <= 0)
    {
        return 0;
    }

    double curr_time = apollo::cyber::Time::Now().ToSecond();

    double v;
    double delta_time;
    double cos_theta;
    double sin_theta;
    Pose2D end_pose;
    double theta;

    // 只更新最后一个obs
    apollo::perception::PerceptionObstacle *obs;

    Polygon2D *obs_local;
    obs_local = get_obs_local_polygon();

    int obs_size = obs_list->perception_obstacle_size();

    obs = obs_list->mutable_perception_obstacle(obs_size - 1);

    if (!obs->has_id())
    {
        return 0;
    }


    v = apollo_sqrt(obs->velocity().x() * obs->velocity().x() +
                 obs->velocity().y() * obs->velocity().y());

    if (up)
    {
        v += 2.0;
    }

    if (low)
    {
        v -= 2.0;

        v = std::max(0.0, v);
    }

    theta = obs->theta();
    if (left)
    {
        theta += 10.0 * M_PI / 180.0;
    }

    if (right)
    {
        theta -= 10.0 * M_PI / 180.0;
    }

    // theta
    obs->set_theta(theta);

    cos_theta = std::cos(theta);
    sin_theta = std::sin(theta);

    end_pose.pos.x = obs->position().x();
    end_pose.pos.y = obs->position().y();

    end_pose.theta = theta;


    // update speed
    obs->mutable_velocity()->set_x(v * cos_theta);
    obs->mutable_velocity()->set_y(v * sin_theta);

    cvt_local_polygon_to_global(&obs_global, obs_local, &end_pose);

    // polygon_point
    obs->clear_polygon_point();
    for (uint j = 0; j < obs_global.vertex_num; ++j)
    {
        common::Point3D *poly = obs->add_polygon_point();
        poly->set_x(obs_global.vertexes[j].x);
        poly->set_y(obs_global.vertexes[j].y);
    }

    return 0;
}


// obs和ego的朝向一致，opencv点是obstacle的中心
int generate_obs_by_cv(
        std::shared_ptr<perception::PerceptionObstacles> &obs_list,
        const Pose2D&pose)
{
    // 清空
    double time_1 = apollo::cyber::Time::Now().ToSecond();
    // perception::PerceptionObstacle obs;
    // po = &obs;
    int size = obs_list->perception_obstacle_size();
    perception::PerceptionObstacle *po = obs_list->add_perception_obstacle();

    // id
    po->set_id(size);

    // center position
    common::Point3D *pt = po->mutable_position();
    pt->set_x(pose.pos.x);
    pt->set_y(pose.pos.y);
    pt->set_z(0);

    // theta
    po->set_theta(pose.theta);

    // velocity
    common::Point3D *vel = po->mutable_velocity();
    vel->set_x(0);
    vel->set_y(0);
    vel->set_z(0);

    // length width height
    const apollo::common::VehicleParam veh_params = get_static_vehicle_params();
    po->set_length(veh_params.length());
    po->set_width(veh_params.width());
    po->set_height(veh_params.height());

    Polygon2D *obs_local = get_obs_local_polygon();
    Polygon2D obs_global;

    cvt_local_polygon_to_global(&obs_global, obs_local, &pose);

    // polygon_point
    for (uint j = 0; j < obs_global.vertex_num; ++j)
    {
        common::Point3D *poly = po->add_polygon_point();
        poly->set_x(obs_global.vertexes[j].x);
        poly->set_y(obs_global.vertexes[j].y);
    }

    // tracking_time s
    po->set_tracking_time(2);

    // Type
    po->set_type(perception::PerceptionObstacle::VEHICLE);
    po->set_sub_type(perception::PerceptionObstacle::ST_CAR);

    // GPS time in seconds.
    po->set_timestamp(time_1);

    // confidence
    po->set_confidence(1);
    po->set_confidence_type(perception::PerceptionObstacle::CONFIDENCE_CNN);

    // trajectory of object
    // po->add_drops()

    // acceleration
    common::Point3D *acc = po->mutable_acceleration();
    acc->set_x(0);
    acc->set_y(0);
    acc->set_z(0);

    obs_list->mutable_header()->set_module_name("perception");
    obs_list->mutable_header()->set_timestamp_sec(time_1);

    return 0;
}

int draw_obs_list(std::shared_ptr<perception::PerceptionObstacles> &obs_list,
                  const Pose2D &base_pose, viz2d_image *window)
{
    Polygon2D obs_global;

    if (obs_list->perception_obstacle_size() <= 0)
    {
        return 0;
    }

    for (int i = 0; i < obs_list->perception_obstacle_size(); i++)
    {
        auto &obs = obs_list->perception_obstacle(i);

        if (!obs.has_id())
        {
            continue;
        }

        // polygon_point
        for (int j = 0; j < obs.polygon_point_size(); ++j)
        {
            auto &point = obs.polygon_point(j);
            obs_global.vertexes[j].x = point.x();
            obs_global.vertexes[j].y = point.y();

            if (j >= 11)
            {
                break;
            }
        }
        obs_global.vertex_num = obs.polygon_point_size();
        obs_global.vertex_num = std::min(obs_global.vertex_num, 12);
        if (obs.polygon_point_size() > 12)
        {
            AWARN << "obs point size is more than 12";
        }

        viz2d_draw_polygon(window, &obs_global,
                                        &base_pose, viz2d_colors_black);
    }

    return 0;
}

int draw_obs_list(perception::PerceptionObstacles *obs_list,
                  const Pose2D&base_pose,  viz2d_image *window)
{
    Polygon2D obs_global;

    if (obs_list->perception_obstacle_size() <= 0)
    {
        return 0;
    }

    CvPoint text_pos;

    CvScalar text_color;

    viz2d_get_color(&text_color, viz2d_colors_yellow);

    CvFont windows_font;

    Position2D center;
    Position2D local;
    // 车辆朝向和speed朝向不一定一致
    Pose2D obs_pose;
    Pose2D speed_pose;

    viz2d_get_cvfont(&windows_font, &(window->font));
    char str[128];

    double v;
    double acc;

    for (int i = 0; i < obs_list->perception_obstacle_size(); i++)
    {
        auto &obs = obs_list->perception_obstacle(i);

        if (!obs.has_id())
        {
            continue;
        }

        // polygon_point
        for (int j = 0; j < obs.polygon_point_size(); ++j)
        {
            auto &point = obs.polygon_point(j);
            obs_global.vertexes[j].x = point.x();
            obs_global.vertexes[j].y = point.y();

            if (j >= 11)
            {
                break;
            }
        }
        obs_global.vertex_num = obs.polygon_point_size();
        obs_global.vertex_num = std::min(obs_global.vertex_num, 12);
        if (obs.polygon_point_size() > 12)
        {
            AWARN << "obs point size is more than 12";
        }

        viz2d_draw_polygon(window, &obs_global,
                                        &base_pose, viz2d_colors_yellow);

        center.x = obs.position().x();
        center.y = obs.position().y();

        // draw veh head direction 
        obs_pose.pos = center;
        obs_pose.theta = obs.theta();

        viz2d_draw_direction(window, &obs_pose, 3, &base_pose,
                            viz2d_colors_red, 2);

        // speed heading
        speed_pose.pos = center;

        double speed_heading;

        speed_heading = std::atan2(obs.velocity().y(), obs.velocity().x());

        speed_pose.theta = speed_heading;

        v = obs.velocity().x() * obs.velocity().x() +
            obs.velocity().y() * obs.velocity().y();

        v = std::sqrt(v);

        if (v > 0.01)
        {
            viz2d_draw_direction(window, &speed_pose, 6, &base_pose,
                                viz2d_colors_yellow, 2);
        }

        // draw state
        cvt_pos_global_to_local(&local, &center, &base_pose);

        acc = obs.acceleration().x() * obs.acceleration().x() +
              obs.acceleration().y() * obs.acceleration().y();

        acc = std::sqrt(acc);

        sprintf(str, "id: %d, v: %.1f m/s, acc: %.1f", obs.id(), v, acc);

        viz2d_get_index(window, &text_pos, local.x, local.y);

        cvPutText(window->image, str, text_pos, &windows_font,
                  text_color);
    }

    return 0;
}



bool remove_obs_by_cv(
        std::shared_ptr<perception::PerceptionObstacles> &obs_list,
        const Position2D &check_pos)
{
    double dist;
    Position2D obs_pos;
    perception::PerceptionObstacle *obs;
    bool remove = false;
    for (int i = 0; i < obs_list->perception_obstacle_size(); i++)
    {
        obs = obs_list->mutable_perception_obstacle(i);

        obs_pos.x = obs->position().x();
        obs_pos.y = obs->position().y();

        dist = calc_point2d_dist(&check_pos, &obs_pos);
        if (dist < 1.0)
        {
            obs_list->mutable_perception_obstacle()->DeleteSubrange(i, 1);
            remove = true;
        }
    }
    return remove;
}

bool remove_obs_by_cv(perception::PerceptionObstacles *obs_list,
                      const Position2D &check_pos)
{
    double dist;
    Position2D obs_pos;
    perception::PerceptionObstacle *obs;
    bool remove = false;
    for (int i = 0; i < obs_list->perception_obstacle_size(); i++)
    {
        obs = obs_list->mutable_perception_obstacle(i);

        obs_pos.x = obs->position().x();
        obs_pos.y = obs->position().y();

        dist = calc_point2d_dist(&check_pos, &obs_pos);
        if (dist < 1.0)
        {
            obs_list->mutable_perception_obstacle()->DeleteSubrange(i, 1);
            remove = true;
        }
    }
    return remove;
}


// obs和ego的朝向一致，opencv点是obstacle的中心
int generate_obs_by_cv(perception::PerceptionObstacles *obs_list,
                       const Pose2D&pose, int virtual_obs_speed_type, double virtual_obs_v)
{
    if (virtual_obs_speed_type < 0)
    {
        return 0;
    }

    // 清空
    double time_1 = apollo::cyber::Time::Now().ToSecond();
    // perception::PerceptionObstacle obs;
    // po = &obs;
    int size = obs_list->perception_obstacle_size();
    perception::PerceptionObstacle *po = obs_list->add_perception_obstacle();

    // id
    po->set_id(size);

    // center position
    common::Point3D *pt = po->mutable_position();
    pt->set_x(pose.pos.x);
    pt->set_y(pose.pos.y);
    pt->set_z(0);

    // theta
    po->set_theta(pose.theta);

    // velocity
    common::Point3D *vel = po->mutable_velocity();

    double speed = 0.0;
    if (virtual_obs_speed_type == 0)
    {
        speed = 0.0;
        vel->set_x(0);
        vel->set_y(0);
        vel->set_z(0);
    }
    else
    {
        speed = virtual_obs_v;
        vel->set_x(speed * std::cos(pose.theta));
        vel->set_y(speed * std::sin(pose.theta));
        vel->set_z(0);
    }

    // length width height
    const apollo::common::VehicleParam &veh_params =
            get_static_vehicle_params();
    po->set_length(veh_params.length());
    po->set_width(veh_params.width());
    po->set_height(veh_params.height());

    Polygon2D *obs_local = get_obs_local_polygon();
    Polygon2D obs_global;

    cvt_local_polygon_to_global(&obs_global, obs_local, &pose);

    // polygon_point
    for (uint j = 0; j < obs_global.vertex_num; ++j)
    {
        common::Point3D *poly = po->add_polygon_point();
        poly->set_x(obs_global.vertexes[j].x);
        poly->set_y(obs_global.vertexes[j].y);
    }

    // tracking_time s
    po->set_tracking_time(2);

    // Type
    po->set_type(perception::PerceptionObstacle::VEHICLE);
    po->set_sub_type(perception::PerceptionObstacle::ST_CAR);

    // GPS time in seconds.
    po->set_timestamp(time_1);

    // confidence
    po->set_confidence(100);
    po->set_confidence_type(perception::PerceptionObstacle::CONFIDENCE_CNN);

    // trajectory of object
    // po->add_drops()

    // acceleration
    common::Point3D *acc = po->mutable_acceleration();
    acc->set_x(0);
    acc->set_y(0);
    acc->set_z(0);

    obs_list->mutable_header()->set_module_name("perception");
    obs_list->mutable_header()->set_timestamp_sec(time_1);

    return 0;
}

/**
 * @brief
 * @note
 *                    o  pose
 *
 *
 *
 *           o-----o------o-------o   route
 *
 *
 *
 * @param  *left_hand_side_id:
 * @param  *right_hand_side_id:
 * @param  *l:
 * @param  *dist_to_right_hand:
 * @param  *pose:
 * @param  &routing_points:
 * @retval
 */
int get_projection_in_route(
        int *left_hand_side_id, int *right_hand_side_id, double *l,
        double *dist_to_right_hand, const Pose2D*pose,
        const std::vector<apollo::hdmap::MapPathPoint> &routing_points)
{
    *left_hand_side_id = -1;
    *right_hand_side_id = -1;

    if (routing_points.size() < 1)
    {
        return 0;
    }

    common::math::Vec2d start;
    start.set_x(pose->pos.x);
    start.set_y(pose->pos.y);

    double min_dist = 1000000;
    double tmp_dist = 1000000;

    int min_idx = -1;

    for (size_t i = 0; i < routing_points.size(); i++)
    {
        const apollo::hdmap::MapPathPoint &point = routing_points.at(i);

        tmp_dist = start.DistanceSquareTo(point);

        if (tmp_dist < min_dist)
        {
            min_dist = tmp_dist;
            min_idx = i;
        }

        // speed up, if min dist is small, and tmp dist is big, then break.
        // this method has a bug in u turn.
        if (min_dist < 2.0 && tmp_dist > 25.0)
        {
            break;
        }
    }

    // 超出两端
    if (min_idx <= 0)
    {
        return 0;
    }
    else if (min_idx >= routing_points.size() - 1)
    {
        return 0;
    }

    // 和路网太远
    if (std::sqrt(min_dist) > 10.0)
    {
        return 0;
    }

    const apollo::hdmap::MapPathPoint &left = routing_points.at(min_idx);
    const apollo::hdmap::MapPathPoint &right = routing_points.at(min_idx + 1);
    common::math::Vec2d route_vec = right;

    route_vec -= left;

    route_vec.Normalize();

    common::math::Vec2d project_point_vec = start;
    project_point_vec -= left;

    double inner = route_vec.InnerProd(project_point_vec);

    // 大于90度
    if (inner < 0.0)
    {
        *left_hand_side_id = min_idx - 1;
        *right_hand_side_id = min_idx;
    }
    else
    {
        *left_hand_side_id = min_idx;
        *right_hand_side_id = min_idx + 1;
    }

    // update offset

    const apollo::hdmap::MapPathPoint &left_point =
            routing_points.at(*left_hand_side_id);
    route_vec = routing_points.at(*right_hand_side_id);
    route_vec -= left_point;

    tmp_dist = route_vec.Length();
    route_vec.Normalize();

    project_point_vec = start;
    project_point_vec -= left_point;
    double offset = route_vec.CrossProd(project_point_vec);

    // update dist
    double dist_to_right;

    dist_to_right = tmp_dist - std::abs(route_vec.InnerProd(project_point_vec));

    *l = offset;

    *dist_to_right_hand = dist_to_right;

    return 0;
}

bool update_pose_by_route(
        Pose2D*end_pose, const Pose2D*start_pose,
        const std::vector<apollo::hdmap::MapPathPoint> &routing_points,
        double move_dist)
{
    int left_hand_side_id;
    int right_hand_side_id;

    double l;
    double dist_to_right;

    // 静止障碍物
    if (move_dist <= 0.000001)
    {
        return false;
    }

    // search start
    get_projection_in_route(&left_hand_side_id, &right_hand_side_id, &l,
                            &dist_to_right, start_pose, routing_points);

    if (left_hand_side_id == -1 || right_hand_side_id == -1)
    {
        return false;
    }

    // search end point
    /**                          move dist
     *                    <--------------------------------->
     *
     *                    o  start pose                     o  end pose
     *
     *
     *
     *           o-----o------o-------o-------o-----------o-----o   route
     *
     */

    if (0)
    {
        double s = dist_to_right;

        int end_pt_idx = -1;

        bool out_of_boundary = true;

        for (size_t i = right_hand_side_id; i < routing_points.size() - 1; i++)
        {
            const apollo::hdmap::MapPathPoint &point = routing_points.at(i);
            const apollo::hdmap::MapPathPoint &next = routing_points.at(i + 1);

            end_pt_idx = i;

            s += point.DistanceTo(next);
            if (s > move_dist)
            {
                out_of_boundary = false;
                s -= point.DistanceTo(next);
                break;
            }
        }

        if (out_of_boundary)
        {
            return false;
        }

        double dist_to_end_point = move_dist - s;

        const apollo::hdmap::MapPathPoint &point =
                routing_points.at(end_pt_idx);
        const apollo::hdmap::MapPathPoint &next =
                routing_points.at(end_pt_idx + 1);

        common::math::Vec2d route_vec = next;
        route_vec -= point;

        route_vec.Normalize();

        common::math::Vec2d route_goal_point = point;
        route_goal_point += dist_to_end_point * route_vec;

        // get offset point
        Pose2D base;
        base.pos.x = route_goal_point.x();
        base.pos.y = route_goal_point.y();

        double heading = route_vec.Angle();

        base.theta = heading;

        Position2D local;

        local.x = -l;
        local.y = 0;

        cvt_pos_local_to_global(&end_pose->pos, &local, &base);

        end_pose->theta = base.theta;
    }
    else
    {
        const apollo::hdmap::MapPathPoint &point =
                routing_points.at(left_hand_side_id);
        const apollo::hdmap::MapPathPoint &next =
                routing_points.at(right_hand_side_id);

        common::math::Vec2d route_vec = next;
        route_vec -= point;

        route_vec.Normalize();

        double heading = route_vec.Angle();

        double cos_theta = std::cos(heading);
        double sin_theta = std::sin(heading);

        end_pose->pos.x = start_pose->pos.x + move_dist * cos_theta;
        end_pose->pos.y = start_pose->pos.y + move_dist * sin_theta;

        end_pose->theta = heading;
    }

    return true;
}

int update_obs_list_pose_by_route(
        perception::PerceptionObstacles *obs_list, const Pose2D&base_pose,
        const std::vector<apollo::hdmap::MapPathPoint> &routing_points_)
{
    if (routing_points_.size() < 1)
    {
        return 0;
    }

    Polygon2D obs_global;

    if (obs_list->perception_obstacle_size() <= 0)
    {
        return 0;
    }

    double curr_time = apollo::cyber::Time::Now().ToSecond();

    double v;
    double delta_time;
    double move_dist;
    double cos_theta;
    double sin_theta;
    Pose2D start_pose;
    Pose2D end_pose;

    double dist_to_ego;

    bool update_obs_by_route = false;

    apollo::perception::PerceptionObstacle *obs;

    Polygon2D *obs_local;
    obs_local = get_obs_local_polygon();

    for (int i = 0; i < obs_list->perception_obstacle_size(); i++)
    {
        obs = obs_list->mutable_perception_obstacle(i);

        if (!obs->has_id())
        {
            continue;
        }

        start_pose.pos.x = obs->position().x();
        start_pose.pos.y = obs->position().y();

        // delete
        dist_to_ego = calc_point2d_dist(&base_pose.pos, &start_pose.pos);
        if (dist_to_ego > 200.0)
        {
            obs_list->mutable_perception_obstacle()->DeleteSubrange(i, 1);
            continue;
        }

        delta_time = curr_time - obs->timestamp();

        v = apollo_sqrt(obs->velocity().x() * obs->velocity().x() +
                     obs->velocity().y() * obs->velocity().y());
        move_dist = delta_time * v;

        // update pose by route
        update_obs_by_route = update_pose_by_route(&end_pose, &start_pose,
                                                   routing_points_, move_dist);

        if (!update_obs_by_route)
        {
            // update pose by move direction
            cos_theta = std::cos(obs->theta());
            sin_theta = std::sin(obs->theta());

            end_pose.pos.x = obs->position().x() + move_dist * cos_theta;
            end_pose.pos.y = obs->position().y() + move_dist * sin_theta;

            end_pose.theta = obs->theta();
        }
        else
        {
            cos_theta = std::cos(end_pose.theta);
            sin_theta = std::sin(end_pose.theta);
        }

        common::Point3D *pt = obs->mutable_position();
        pt->set_x(end_pose.pos.x);
        pt->set_y(end_pose.pos.y);
        pt->set_z(0);

        // theta
        obs->set_theta(end_pose.theta);

        // update speed
        obs->mutable_velocity()->set_x(v * cos_theta);
        obs->mutable_velocity()->set_y(v * sin_theta);

        cvt_local_polygon_to_global(&obs_global, obs_local, &end_pose);

        // polygon_point
        obs->clear_polygon_point();
        for (uint j = 0; j < obs_global.vertex_num; ++j)
        {
            common::Point3D *poly = obs->add_polygon_point();
            poly->set_x(obs_global.vertexes[j].x);
            poly->set_y(obs_global.vertexes[j].y);
        }

        // update time
        obs->set_timestamp(curr_time);
    }

    obs_list->mutable_header()->set_module_name("perception");
    obs_list->mutable_header()->set_timestamp_sec(curr_time);

    return 0;
}

}