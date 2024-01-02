#include "viz3d_component.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "cyber/common/global_data.h"
#include "cyber/proto/run_mode_conf.pb.h"

#include "modules/planning/common/planning_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"

#include "modules/viz2d/viz2d_map_elements.h"
#include "draw_path.h"


namespace apollo
{
viz3d_component::viz3d_component(/* args */)
{
    node_ = apollo::cyber::CreateNode(apollo::cyber::binary::GetName());

    map_service.reset(new apollo::dreamview::MapService(false));
}

int viz3d_component::init()
{
    obs_writer_ = node_->CreateWriter<apollo::perception::PerceptionObstacles>(
            FLAGS_hmi_obstacle_topic);

    drive_mode_writer_ = node_->CreateWriter<apollo::canbus::Chassis>(
            FLAGS_chassis_drive_mode_topic);

    debug_mode_writer_ = node_->CreateWriter<apollo::cyber::proto::DebugMsg>(
            FLAGS_debug_planning_msg);

    traffic_light_detection_writer_ =
            node_->CreateWriter<perception::TrafficLightDetection>(
                    FLAGS_traffic_light_detection_topic);

    lane_borrow_manual_writer_ =
            node_->CreateWriter<planning::LaneBorrowManual>(
                    "lane_borrow_manual");

    chassis_reader_ = node_->CreateReader<canbus::Chassis>(
            FLAGS_chassis_topic,
            [this](const std::shared_ptr<canbus::Chassis>& chassis) {
                ADEBUG << "Received chassis data: run chassis callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                chassis_.CopyFrom(*chassis);
            });

    control_cmd_reader_ = node_->CreateReader<control::ControlCommand>(
            FLAGS_control_command_topic,
            [this](const std::shared_ptr<control::ControlCommand>& cmd) {
                ADEBUG << "Received control data: run control callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                control_cmd_.CopyFrom(*cmd);
            });

    localization_reader_ = node_->CreateReader<
            localization::LocalizationEstimate>(
            FLAGS_localization_topic,
            [this](const std::shared_ptr<localization::LocalizationEstimate>&
                           localization) {
                ADEBUG << "Received chassis data: run chassis callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                localization_.CopyFrom(*localization);
            });

    trajectory_reader_ = node_->CreateReader<planning::ADCTrajectory>(
            FLAGS_planning_trajectory_topic,
            [this](const std::shared_ptr<planning::ADCTrajectory>& trajectory) {
                ADEBUG << "Received chassis data: run chassis callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                trajectory_.CopyFrom(*trajectory);
            });

    prediction_reader_ = node_->CreateReader<prediction::PredictionObstacles>(
            FLAGS_prediction_topic,
            [this](const std::shared_ptr<prediction::PredictionObstacles>&
                           obstacles) {
                ADEBUG << "Received prediction data: run callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                prediction_.CopyFrom(*obstacles);
            });

    routing_response_reader_ = node_->CreateReader<routing::RoutingResponse>(
            FLAGS_routing_response_topic,
            [this](const std::shared_ptr<routing::RoutingResponse>& routing) {
                ADEBUG << "Received routing data: run callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                route_.CopyFrom(*routing);
            });

    perception_reader_ = node_->CreateReader<perception::PerceptionObstacles>(
            FLAGS_perception_obstacle_topic,
            [this](const std::shared_ptr<perception::PerceptionObstacles>&
                           perception_obstacles)
            {
                ADEBUG << "Received perception data: run perception callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                perception_.CopyFrom(*perception_obstacles);
            });

    play_state_reader_ = node_->CreateReader<apollo::cyber::proto::PlayInfo>(
            "channel/play_info",
            [this](const std::shared_ptr<apollo::cyber::proto::PlayInfo>& play)
            {
                ADEBUG << "Received perception data: run perception callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                play_info_.CopyFrom(*play);
            });

    if (FLAGS_replay_mode)
    {
        hmi_perception_reader_ =
                node_->CreateReader<apollo::perception::PerceptionObstacles>(
                        FLAGS_hmi_obstacle_topic,
                        [this](const std::shared_ptr<
                                apollo::perception::PerceptionObstacles>& play)
                        {
                            ADEBUG << "Received perception data: run "
                                      "perception callback.";
                            std::lock_guard<std::mutex> lock(mutex_);
                            hmi_perception_.CopyFrom(*play);
                        });

        AINFO << "replay mode";
    }

    play_info_.set_ratio(-1);

    AINFO << "open cv viz begin";


    AINFO << "planning viz init finish";

    AINFO << "hmap_window2d_init finish";


    vehicle_config = apollo::common::VehicleConfigHelper::GetConfig();

    veh_local_polygon = init_adv_box(vehicle_config.vehicle_param());

    get_hdmap_center_base(&hmap_base_pose);

    wheel_base = vehicle_config.vehicle_param().wheel_base();

    debug_mode.pause_debug.enabled = false;
    key_value_ = -1;
    virtual_obs_speed_type_ = -1;

    virtual_obs_type_ = perception::PerceptionObstacle::Type::
            PerceptionObstacle_Type_UNKNOWN;

    hmi_perception_.Clear();

    drive_mode_ = apollo::canbus::Chassis::COMPLETE_MANUAL;

    debug_msg_.set_debug_mode(apollo::cyber::proto::DebugMode::none);

    // set initial vehicle state by cmd
    // need to sleep, because advertised channel is not ready immediately
    // simple test shows a short delay of 80 ms or so
    AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    AINFO << "viz component init finish";

    auto& global_conf = apollo::cyber::common::GlobalData::Instance()->Config();

    run_mode_ = global_conf.run_mode_conf().run_mode();

    // todo: unify run mode
    if(run_mode_ == apollo::cyber::proto::RunMode::MODE_REALITY)
    {
        module_state_.run_mode = ApolloRunMode::reality;
    }
    else
    {
        module_state_.run_mode = ApolloRunMode::simple_simulation;
    }

    manual_traffic_light_generator_.Init();
    manual_traffic_light_color_ = perception::TrafficLight::UNKNOWN;

    decision_safe_buffer_ = FLAGS_static_obstacle_nudge_l_buffer;

    path_bound_safe_buffer_ = FLAGS_obstacle_lat_buffer;

    lane_borrow_manual_changed_ = false;

    window_ = new pcl::visualization::PCLVisualizer("Viewer");

    window_->addCoordinateSystem(1.0);

    pcl_color back_color;
    pcl_get_color(&back_color, pcl_colors_dark_blue);
    window_->setBackgroundColor(back_color.r, back_color.g, back_color.b);

    return 0;
}

int viz3d_component::close()
{
    window_->close();
    delete window_;
    return 0;
}

int viz3d_component::data_tanslate(double max_steering_wheel_angle_,
                                   apollo::planning::DiscretizedPath& lat_path)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);

        viz_subscribe_.chassis.Clear();
        viz_subscribe_.chassis.CopyFrom(chassis_);

        viz_subscribe_.localization_estimate.Clear();
        viz_subscribe_.localization_estimate.CopyFrom(localization_);

        viz_subscribe_.traj.Clear();
        viz_subscribe_.traj.CopyFrom(trajectory_);

        viz_subscribe_.prediction_obstacles.Clear();
        viz_subscribe_.prediction_obstacles.CopyFrom(prediction_);

        viz_subscribe_.control.Clear();
        viz_subscribe_.control.CopyFrom(control_cmd_);

        is_new_route_ = false;

        is_new_route_ =
                hdmap::PncMap::IsNewRouting(viz_subscribe_.routing, route_);

        if (is_new_route_)
        {
            viz_subscribe_.routing.Clear();
            viz_subscribe_.routing.CopyFrom(route_);

            AINFO << "receive new routing response";
        }

        viz_subscribe_.perception.Clear();
        viz_subscribe_.perception.CopyFrom(perception_);
    }

    veh_pose_ = viz_subscribe_.localization_estimate.pose();

    double time = apollo::cyber::Time::Now().ToSecond();

    double delta_time = 1000;

    delta_time = 1000;
    if (viz_subscribe_.localization_estimate.has_header())
    {
        delta_time =
                time -
                viz_subscribe_.localization_estimate.header().timestamp_sec();
    }

    if (delta_time > 0.1)
    {
        SystemStateSet(module_state_, ModuleName::localization,
                       ModuleStateCodeError);
    }
    else
    {
        SystemStateSet(module_state_, ModuleName::localization,
                       ModuleStateCodeSuccess);
    }

    // routing, 一般routing不会每一帧都改变，所以这里只需要计算一次
    // route是由相邻车道，前后连接车道组成

    if (is_new_route_)
    {

        double dist;
        const apollo::hdmap::MapPathPoint* histroy_point;
        if (viz_subscribe_.routing.road_size() > 0)
        {
            route_path_list_.clear();
            map_service->GetPathsFromRouting(viz_subscribe_.routing,
                                             &route_path_list_);

            AINFO << "route size " << viz_subscribe_.routing.road_size()
                  << ", path size " << route_path_list_.size();

            for (const apollo::hdmap::Path& path : route_path_list_)
            {
                // Downsample the path points for frontend display.

                const std::vector<apollo::hdmap::MapPathPoint>& points =
                        path.path_points();

                histroy_point = nullptr;
                for (std::size_t i = 0; i < points.size(); i++)
                {
                    const apollo::hdmap::MapPathPoint& point = points.at(i);

                    if (histroy_point != nullptr)
                    {
                        dist = point.DistanceTo(*histroy_point);

                        if (dist < 1.0)
                        {
                            continue;
                        }
                    }
                    histroy_point = &points.at(i);
                    routing_points_.emplace_back(point);
                }
            }
        }
    }

    if (viz_subscribe_.routing.has_header())
    {
        SystemStateSet(module_state_, ModuleName::routing,
                       ModuleStateCodeSuccess);
    }
    else
    {
        SystemStateSet(module_state_, ModuleName::routing,
                       ModuleStateCodeError);
    }

    // path

    for (int i = 0; i < viz_subscribe_.traj.path_point_size(); i++)
    {
        lat_path.emplace_back(viz_subscribe_.traj.path_point().at(i));
    }

    delta_time = 1000;
    if (viz_subscribe_.traj.has_header())
    {
        delta_time = time - viz_subscribe_.traj.header().timestamp_sec();
    }

    if (delta_time > 0.3)
    {
        SystemStateSet(module_state_, ModuleName::planner, ModuleStateCodeError);
    }
    else
    {
        SystemStateSet(module_state_, ModuleName::planner,
                       ModuleStateCodeSuccess);
    }

    // obstacles trajectory
    obstacles_trajectory_points.clear();
    Position2D point;
    for (int i = 0;
         i < viz_subscribe_.prediction_obstacles.prediction_obstacle_size();
         ++i)
    {
        // std::cout << "width: " <<
        // pop->prediction_obstacle(i).perception_obstacle().width() <<
        // "\n"; std::cout << "height: " <<
        // pop->prediction_obstacle(i).perception_obstacle().height() <<
        // "\n";

        // 一个障碍物可能有多个轨迹

        const apollo::prediction::PredictionObstacle& obs =
                viz_subscribe_.prediction_obstacles.prediction_obstacle(i);

        for (int j = 0; j < obs.trajectory().size(); ++j)
        {
            std::vector<Position2D> ob_tra_pts;

            const prediction::Trajectory& traj = obs.trajectory(j);

            for (int k = 0; k < traj.trajectory_point_size(); ++k)
            {
                
                point.x = traj.trajectory_point(k).path_point().x();
                point.y = traj.trajectory_point(k).path_point().y();
                ob_tra_pts.push_back(point);
            }

            obstacles_trajectory_points.push_back(ob_tra_pts);
        }
    }

    delta_time = 1000;
    if (viz_subscribe_.prediction_obstacles.has_header())
    {
        delta_time =
                time -
                viz_subscribe_.prediction_obstacles.header().timestamp_sec();
    }

    if (delta_time > 0.3)
    {
        SystemStateSet(module_state_, ModuleName::prediction,
                       ModuleStateCodeError);
    }
    else
    {
        SystemStateSet(module_state_, ModuleName::prediction,
                       ModuleStateCodeSuccess);
    }

    // chassis

    double steering_angle;
    steering_angle = viz_subscribe_.chassis.steering_percentage() / 100.0 *
                     max_steering_wheel_angle_;

    chassis_data.set_x(viz_subscribe_.chassis.speed_mps());
    chassis_data.set_y(steering_angle);

    delta_time = 1000;
    if (viz_subscribe_.chassis.has_header())
    {
        delta_time = time - viz_subscribe_.chassis.header().timestamp_sec();
    }

    if (delta_time > 0.1)
    {
        SystemStateSet(module_state_, ModuleName::chassis,
                       ModuleStateCodeError);
    }
    else
    {
        SystemStateSet(module_state_, ModuleName::chassis,
                       ModuleStateCodeSuccess);
    }

    // control

    double steering_wheel_angle;
    steering_wheel_angle = viz_subscribe_.control.steering_target() / 100 *
                           vehicle_config.vehicle_param().max_steer_angle();

    steering_wheel_angle = steering_wheel_angle * 180.0 / M_PI;

    // 角度,acc
    control_data_.set_x(steering_wheel_angle);
    control_data_.set_y(viz_subscribe_.control.acceleration());

    delta_time = 1000;
    if (viz_subscribe_.control.has_header())
    {
        delta_time = time - viz_subscribe_.control.header().timestamp_sec();
    }

    if (delta_time > 0.1)
    {
        SystemStateSet(module_state_, ModuleName::control,
                       ModuleStateCodeError);
    }
    else
    {
        SystemStateSet(module_state_, ModuleName::control,
                       ModuleStateCodeSuccess);
    }

    // perception

    delta_time = 1000;
    if (viz_subscribe_.perception.has_header())
    {
        delta_time = time - viz_subscribe_.perception.header().timestamp_sec();
    }

    if (delta_time > 0.1)
    {
        SystemStateSet(module_state_, ModuleName::perception,
                       ModuleStateCodeError);
    }
    else
    {
        SystemStateSet(module_state_, ModuleName::perception,
                       ModuleStateCodeSuccess);
    }

    return 0;
}

int viz3d_component::process(double max_steering_wheel_angle_)
{

    apollo::planning::DiscretizedPath lateral_path_;

    data_tanslate(max_steering_wheel_angle_, lateral_path_);

    veh_global_pose.pos.x = veh_pose_.position().x();
    veh_global_pose.pos.y = veh_pose_.position().y();
    veh_global_pose.theta = veh_pose_.heading();

    cvt_local_polygon_to_global(&veh_global_polygon, &veh_local_polygon,
                                &veh_global_pose);

    // draw planning
    pcl_color color;
    pcl_get_color(&color, pcl_colors_cyan);
    draw_trajectory(window_, &viz_subscribe_.traj, color,
                    &veh_global_pose, &veh_local_polygon);

    // viz2d_draw_xy_axis(main_window_);
    // viz2d_draw_xy_axis(hmap_window_);

    // // draw localization polygon
    // cv_draw_polygon(main_window_, &veh_global_polygon, &veh_global_pose,
    //                 viz2d_colors_orange, 2);

    // // draw safe buffer, 注意：横向决策buffer 和纵向决策buffer是一致的。path
    // // bound safe buffer是FLAGS_obstacle_lat_buffer
    // Polygon2D safe_adc_local_polygon;
    // Polygon2D safe_adc_global_polygon;
    // extend_adv_box_by_width(&safe_adc_local_polygon, decision_safe_buffer_,
    //                         &veh_local_polygon);

    // cvt_local_polygon_to_global(&safe_adc_global_polygon,
    //                             &safe_adc_local_polygon, &veh_global_pose);

    // viz2d_draw_polygon(main_window_, &safe_adc_global_polygon,
    //                                 &veh_global_pose, viz2d_colors_orange);

    // // hdmap
    // // viz2d_draw_hdmap(viz2d, &veh_global_pose, false);
    // viz2d_draw_simple_hdmap(main_window_, &veh_global_pose);

    // // draw full hmap in map window
    // // viz2d_draw_hdmap(hmap_viz2d, &hmap_base_pose, true);
    // viz2d_draw_full_simple_hdmap(hmap_window_, &hmap_base_pose);

    // int ret;
    // // ret = viz2d_draw_route(viz2d, routing_points_, &veh_global_pose,
    // //                       viz2d_colors_green, 2);
    // ret = viz2d_draw_route2(main_window_, route_path_list_, &veh_global_pose,
    //                       viz2d_colors_green, 2);
    // if (ret < 0)
    // {
    //     AERROR << "fail to draw route";
    // }

    // Position2D waypoint;
    // for (size_t i = 0;
    //      i < viz_subscribe_.routing.routing_request().waypoint_size(); i++)
    // {
    //     waypoint.x =
    //             viz_subscribe_.routing.routing_request().waypoint(i).pose().x();
    //     waypoint.y =
    //             viz_subscribe_.routing.routing_request().waypoint(i).pose().y();
    //     viz2d_draw_circle_wrapper(main_window_, &waypoint, &veh_global_pose,
    //                              viz2d_colors_green, 8, true);
    // }

    // // draw route in hmap window
    // viz2d_draw_route2(hmap_window_, route_path_list_, &hmap_base_pose,
    //                 viz2d_colors_cyan, 3);
    // // viz2d_draw_route(hmap_viz2d, routing_points_, &hmap_base_pose,
    // //                 viz2d_colors_cyan, 3);

    // // draw vehicle position in map window
    // viz2d_draw_circle_wrapper(hmap_window_, &veh_global_pose.pos, &hmap_base_pose,
    //                          viz2d_colors_red, 10, true);

    // viz2d_draw_direction(hmap_window_, &veh_global_pose, 60, &hmap_base_pose,
    //                     viz2d_colors_red, 2);

    // viz2d_draw_crosswalk(main_window_, &veh_global_pose, viz2d_colors_pink, 400, 2);

    // // reference line

    // viz2d_draw_ref_line(main_window_, &ref_lines_, &veh_global_pose, viz2d_colors_green);

    // // draw prediction
    // viz2d_draw_prediction(main_window_, obstacles_trajectory_points, &veh_global_pose,
    //                      viz2d_colors_yellow, 2);

    // // draw chassis

    // viz2d_draw_chassis_feedback(main_window_, chassis_data.x(), chassis_data.y());

    // // draw control cmd

    // viz2d_draw_control_commond_info(main_window_, control_data_.y(), control_data_.x());

    // // draw bev rtk status
    // viz2d_draw_rtk_state(main_window_, viz_subscribe_.localization_estimate,
    //                     &veh_global_pose);

    // viz2d_draw_system_state(main_window_, module_state_, &veh_global_pose);

    // // control front wheel

    // double steering_wheel_angle = control_data_.x();

    // front_wheel_angle =
    //         steering_wheel_angle / vehicle_config.vehicle_param().steer_ratio();
    // front_wheel_angle = front_wheel_angle * M_PI / 180.0;
    // viz2d_draw_front_wheel_state(main_window_, &veh_global_pose, wheel_base,
    //                             front_wheel_angle, viz2d_colors_pink);

    // // std::cout << "control front wheel " << front_wheel_angle * 180 / M_PI
    // //   << "\n";

    // // chassis front wheel

    // steering_wheel_angle = chassis_data.y();

    // front_wheel_angle =
    //         steering_wheel_angle / vehicle_config.vehicle_param().steer_ratio();
    // front_wheel_angle = front_wheel_angle * M_PI / 180.0;
    // viz2d_draw_front_wheel_state(main_window_, &veh_global_pose, wheel_base,
    //                             front_wheel_angle, viz2d_colors_red);

    // // get key
    // key_value_ = cv::waitKey(30);  // 50 mill second delay

    // if (key_value_ >= 0)
    // {
    //     AINFO << "key is: " << key_value_;
    // }

    // drive_mode_changed_ = false;
    // bool debug_mode_changed_ = false;

    // manual_traffic_light_color_changed_ = false;

    // lane_borrow_manual_changed_ = false;

    // cv_direction_key direction_keys;
    // cv_key_direction_init(&direction_keys);

    // switch (key_value_)
    // {
    //     case 'p':
    //         if (debug_mode.pause_debug.enabled)
    //         {
    //             debug_mode.pause_debug.enabled = false;

    //             AINFO << "p key is be pressed, and system will continue";
    //         }
    //         else
    //         {
    //             debug_mode.pause_debug.enabled = true;
    //             AINFO << "p key is be pressed, and system will pause";
    //         }

    //         if (debug_mode.pause_debug.enabled)
    //         {
    //             debug_msg_.set_debug_mode(cyber::proto::DebugMode::pause);
    //         }
    //         else
    //         {
    //             debug_msg_.set_debug_mode(cyber::proto::DebugMode::none);
    //         }
    //         debug_mode_changed_ = true;
    //         break;

    //     case 'r':
    //         if (drive_mode_ == apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE)
    //         {
    //             drive_mode_ = apollo::canbus::Chassis::COMPLETE_MANUAL;

    //             AINFO << "r key is be pressed, and system will stop";
    //         }
    //         else
    //         {
    //             drive_mode_ = apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE;

    //             AINFO << "r key is be pressed, and system will start";

    //         }

    //         drive_mode_changed_ = true;
    //         break;
    //     case 's':
    //         virtual_obs_speed_type_ = 0;
    //         break;
    //     case 'd':
    //         virtual_obs_speed_type_ = 1;
    //         break;
    //     case 'c':
    //         virtual_obs_speed_type_ = -1;
    //         hmi_perception_.clear_perception_obstacle();
    //         break;
    //     case 'l':

    //         if (lane_borrow_manual_.has_lane_borrow_by_manual() &&
    //             lane_borrow_manual_.lane_borrow_by_manual())
    //         {
    //             lane_borrow_manual_.set_lane_borrow_by_manual(false);
    //             lane_borrow_manual_.clear_lane_borrow_dir();
    //         }
    //         else
    //         {
    //             lane_borrow_manual_.set_lane_borrow_by_manual(true);
    //             lane_borrow_manual_.set_lane_borrow_dir(
    //                     planning::LaneBorrowDirection::
    //                             LANE_BORROW_DIRECTION_LEFT);
    //         }
    //         lane_borrow_manual_changed_ = true;
    //         break;
    //     case 82:

    //         direction_keys.up = true;
    //         break;
    //     case 81:

    //         direction_keys.left = true;
    //         break;
    //     case 83:

    //         direction_keys.right = true;
    //         break;
    //     case 84:

    //         direction_keys.low = true;
    //         break;
    //     case 'i':

    //         if (lane_borrow_manual_.has_lane_borrow_by_manual() &&
    //             lane_borrow_manual_.lane_borrow_by_manual())
    //         {
    //             lane_borrow_manual_.set_lane_borrow_by_manual(false);
    //             lane_borrow_manual_.clear_lane_borrow_dir();
    //         }
    //         else
    //         {
    //             lane_borrow_manual_.set_lane_borrow_by_manual(true);
    //             lane_borrow_manual_.set_lane_borrow_dir(
    //                     planning::LaneBorrowDirection::
    //                             LANE_BORROW_DIRECTION_RIGHT);
    //         }
    //         lane_borrow_manual_changed_ = true;
    //         break;

    //     case 't':
    //         switch (manual_traffic_light_color_)
    //         {
    //             case perception::TrafficLight::UNKNOWN:
    //                 manual_traffic_light_color_ = perception::TrafficLight::RED;
    //                 break;
    //             case perception::TrafficLight::RED:
    //                 manual_traffic_light_color_ =
    //                         perception::TrafficLight::YELLOW;
    //                 break;
    //             case perception::TrafficLight::YELLOW:
    //                 manual_traffic_light_color_ =
    //                         perception::TrafficLight::GREEN;
    //                 break;
    //             case perception::TrafficLight::GREEN:
    //                 manual_traffic_light_color_ =
    //                         perception::TrafficLight::BLACK;
    //                 break;
    //             case perception::TrafficLight::BLACK:
    //             default:
    //                 manual_traffic_light_color_ = perception::TrafficLight::RED;
    //                 break;
    //         }

    //         AINFO << "traffic light color: " << manual_traffic_light_color_;

    //         manual_traffic_light_color_changed_ = true;
    //         break;
    //     default:
    //         break;
    // }

    // viz2d_draw_pause_state(main_window_, debug_mode);

    // viz2d_draw_drive_mode(main_window_, drive_mode_);

    // viz2d_draw_run_mode(main_window_, module_state_.run_mode);

    // double time_stamp = -1.0;

    // if (viz_subscribe_.localization_estimate.has_header())
    // {
    //     if (viz_subscribe_.localization_estimate.header().has_timestamp_sec())
    //     {
    //         time_stamp = viz_subscribe_.localization_estimate.header()
    //                              .timestamp_sec();
    //     }
    // }

    // if (time_stamp < 0)
    // {
    //     time_stamp = apollo::cyber::Clock::NowInSeconds();
    // }

    // viz2d_draw_localization_time(main_window_, time_stamp);

    // // draw ratio for replay

    // viz2d_draw_replay_info(main_window_, play_info_.ratio());

    // // process mouse cmd

    // cvSetMouseCallback(main_window_->win_name, on_Mouse);

    // Position2D local_position;

    // ret = transform_cv_point_to_vrf_point(main_window_, &local_position);
    // if (ret >= 0 && !FLAGS_replay_mode)
    // {
    //     Pose2D obs_global_pose;
    //     cvt_pos_local_to_global(&obs_global_pose.pos, &local_position,
    //                             &veh_global_pose);

    //     // forward
    //     obs_global_pose.theta = veh_global_pose.theta;

    //     // reverse
    //     // obs_global_pose.theta = veh_global_pose.theta + M_PI;

    //     // 如果鼠标位置和历史上的位置比较接近，就删除一个历史的障碍物
    //     bool remove_obs;
    //     remove_obs = remove_obs_by_cv(&hmi_perception_, obs_global_pose.pos);

    //     // generate apollo obstacle
    //     if (!remove_obs)
    //     {
    //         DisplayConfig* window_config = get_windows2d_config();

    //         generate_obs_by_cv(&hmi_perception_, obs_global_pose,
    //                            virtual_obs_speed_type_,
    //                            window_config->virtual_obs_speed());

    //         // reset
    //         virtual_obs_speed_type_ = -1;
    //     }
    // }

    // if (!FLAGS_replay_mode)
    // {
    //     if (0)
    //     {
    //         update_obs_list_pose_by_route(&hmi_perception_, veh_global_pose,
    //                                       routing_points_);
    //     }
    //     else
    //     {
    //         if (has_cv_direction_key(&direction_keys))
    //         {
    //             update_obs_list_pose_by_key(
    //                     &hmi_perception_, direction_keys.left,
    //                     direction_keys.right, direction_keys.up,
    //                     direction_keys.low);
    //         }

    //         update_obs_list_pose_by_heading(&hmi_perception_, veh_global_pose);
    //     }
    // }

    // // apollo perception
    // draw_obs_list(&hmi_perception_, veh_global_pose,main_window_);
    // AINFO << "hmi perception size "
    //       << hmi_perception_.perception_obstacle_size();

    // draw_obs_list(&viz_subscribe_.perception, veh_global_pose,main_window_);

    // viz2d_draw_path(main_window_, lateral_path_, &veh_global_pose,
    //                 viz2d_colors_purple, 4);

    // viz2d_draw_grid(main_window_, 100, 100, 100, 100, &veh_global_pose,
    //                 viz2d_colors_gray, 1);

    // history_control_data_ = control_data_;

    // // 如果是replay模式,不需要写
    // if (!FLAGS_replay_mode)
    // {
    //     obs_writer_->Write(hmi_perception_);
    // }

    // // publish drive mode, subscribed by chassis
    // if (drive_mode_changed_)
    // {
    //     apollo::canbus::Chassis chassis_drive_mode;
    //     chassis_drive_mode.set_driving_mode(drive_mode_);
    //     drive_mode_writer_->Write(chassis_drive_mode);
    // }

    // if (debug_mode_changed_)
    // {
    //     debug_mode_writer_->Write(debug_msg_);
    // }

    // // decision

    // if (viz_subscribe_.traj.has_decision())
    // {
    //     viz_draw_decision_list(viz_subscribe_.traj.mutable_decision(), main_window_,
    //                            &veh_global_pose);

    //     viz_draw_lane_decision(viz_subscribe_.traj.mutable_decision(), main_window_,
    //                            &veh_global_pose, viz_subscribe_.traj);
    // }

    // // draw scenario recognize
    // if (viz_subscribe_.traj.has_speed_decision())
    // {
    //     viz_draw_dynamic_scenerio_type(main_window_, &veh_global_pose,
    //                                    viz_subscribe_.traj.speed_decision());
    // }

    // manual_traffic_light_generator_.Proc(viz_subscribe_.localization_estimate,
    //                                      &manual_traffic_light_,
    //                                      manual_traffic_light_color_);

    // // 检查green light是否应该结束
    // if (manual_traffic_light_generator_.is_green_light_over())
    // {
    //     manual_traffic_light_color_ = perception::TrafficLight::RED;
    // }

    // viz_draw_traffic_lights(&manual_traffic_light_, &veh_global_pose, main_window_);

    // // light size > 0，才发布
    // if (manual_traffic_light_.has_header() &&
    //     manual_traffic_light_.traffic_light_size() > 0)
    // {
    //     traffic_light_detection_writer_->Write(manual_traffic_light_);
    // }

    // if (lane_borrow_manual_changed_)
    // {
    //     lane_borrow_manual_writer_->Write(lane_borrow_manual_);
    // }

    // viz2d_show_result_in_per_frame(main_window_);
    // viz2d_show_result_in_per_frame(hmap_window_);

    window_->spinOnce(30);

    return 0;
}

}  // namespace apollo