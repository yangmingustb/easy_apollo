#include "viz3d_component.h"
#include "cyber/common/global_data.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo
{

viz3d_component::viz3d_component(/* args */)
{
    node_ = apollo::cyber::CreateNode(apollo::cyber::binary::GetName());
}

int viz3d_component::init()
{
    obs_writer_ = node_->CreateWriter<apollo::perception::PerceptionObstacles>(
            FLAGS_hmi_obstacle_topic);

    drive_mode_writer_ = node_->CreateWriter<apollo::canbus::Chassis>(
            FLAGS_chassis_drive_mode_topic);

    chassis_reader_ = node_->CreateReader<canbus::Chassis>(
            FLAGS_chassis_topic,
            [this](const std::shared_ptr<canbus::Chassis>& chassis)
            {
                ADEBUG << "Received chassis data: run chassis callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                chassis_.CopyFrom(*chassis);
            });

    control_cmd_reader_ = node_->CreateReader<control::ControlCommand>(
            FLAGS_control_command_topic,
            [this](const std::shared_ptr<control::ControlCommand>& cmd)
            {
                ADEBUG << "Received control data: run control callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                control_cmd_.CopyFrom(*cmd);
            });

    localization_reader_ = node_->CreateReader<
            localization::LocalizationEstimate>(
            FLAGS_localization_topic,
            [this](const std::shared_ptr<localization::LocalizationEstimate>&
                           localization)
            {
                ADEBUG << "Received chassis data: run chassis callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                localization_.CopyFrom(*localization);
            });

    trajectory_reader_ = node_->CreateReader<planning::ADCTrajectory>(
            FLAGS_planning_trajectory_topic,
            [this](const std::shared_ptr<planning::ADCTrajectory>& trajectory)
            {
                ADEBUG << "Received chassis data: run chassis callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                trajectory_.CopyFrom(*trajectory);
            });

    prediction_reader_ = node_->CreateReader<prediction::PredictionObstacles>(
            FLAGS_prediction_topic,
            [this](const std::shared_ptr<prediction::PredictionObstacles>&
                           obstacles)
            {
                ADEBUG << "Received prediction data: run callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                prediction_.CopyFrom(*obstacles);
            });

    routing_response_reader_ = node_->CreateReader<routing::RoutingResponse>(
            FLAGS_routing_response_topic,
            [this](const std::shared_ptr<routing::RoutingResponse>& routing)
            {
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

    auto& global_conf = apollo::cyber::common::GlobalData::Instance()->Config();

    run_mode_ = global_conf.run_mode_conf().run_mode();

    // todo: unify run mode
    if (run_mode_ == apollo::cyber::proto::RunMode::MODE_REALITY)
    {
        module_state_.run_mode = ApolloRunMode::reality;
    }
    else
    {
        module_state_.run_mode = ApolloRunMode::simple_simulation;
    }

    AINFO << "open cv viz begin";

    AINFO << "planning viz init finish";

    AINFO << "hmap_uviz_init finish";


    vehicle_config = apollo::common::VehicleConfigHelper::GetConfig();

    // 方向盘最大角度
    max_steering_wheel_angle_ =
            vehicle_config.vehicle_param().max_steer_angle();

    max_steering_wheel_angle_ = max_steering_wheel_angle_ * 180 / M_PI;

    veh_local_polygon = init_adv_box(vehicle_config.vehicle_param());

    wheel_base = vehicle_config.vehicle_param().wheel_base();

    debug_mode.pause_debug.enabled = false;
    key_value = -1;

    hmi_perception_.Clear();

    drive_mode_ = apollo::canbus::Chassis::DrivingMode::
            Chassis_DrivingMode_COMPLETE_MANUAL;

    // set initial vehicle state by cmd
    // need to sleep, because advertised channel is not ready immediately
    // simple test shows a short delay of 80 ms or so
    AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    AINFO << "viz component init finish";

    return 0;
}

int viz3d_component::close()
{

    return 0;
}

int viz3d_component::data_tanslate()
{
    AINFO << "parse begin";

    trajectory_points.clear();
    obstacles_trajectory_points.clear();
    lateral_path_.clear();

    veh_pose_ = localization_.pose();

    AINFO << "localization";

    veh_global_pose.pos.x = veh_pose_.position().x();
    veh_global_pose.pos.y = veh_pose_.position().y();
    veh_global_pose.theta = veh_pose_.heading();

    // trajectory
    trajectory_points.clear();
    if (!trajectory_.trajectory_point().empty())
    {
        for (int i = 0; i < trajectory_.trajectory_point_size(); ++i)
        {
            auto& tp = trajectory_.trajectory_point(i);
            cv::Vec<double, 10> pt = {tp.v(),
                                      tp.a(),
                                      tp.relative_time(),
                                      tp.path_point().x(),
                                      tp.path_point().y(),
                                      tp.path_point().theta(),
                                      tp.path_point().kappa(),
                                      tp.path_point().s(),
                                      tp.path_point().dkappa(),
                                      tp.path_point().ddkappa()};

            trajectory_points.push_back(pt);
        }
    }

    AINFO << "traj";

    // path

    if (!trajectory_.path_point().empty())
    {
        for (int i = 0; i < trajectory_.path_point_size(); i++)
        {
            lateral_path_.push_back(trajectory_.path_point().at(i));
        }
    }

    AINFO << "path";

    AINFO << "lateral path " << lateral_path_.size();

    // obstacles trajectory
    obstacles_trajectory_points.clear();
    for (int i = 0; i < prediction_.prediction_obstacle_size(); ++i)
    {
        // std::cout << "width: " <<
        // pop->prediction_obstacle(i).perception_obstacle().width() <<
        // "\n"; std::cout << "height: " <<
        // pop->prediction_obstacle(i).perception_obstacle().height() <<
        // "\n";

        // 一个障碍物可能有多个轨迹

        const apollo::prediction::PredictionObstacle& obs =
                prediction_.prediction_obstacle(i);

        for (int j = 0; j < obs.trajectory().size(); ++j)
        {
            std::vector<cv::Point2d> ob_tra_pts;

            for (int k = 0; k < obs.trajectory(j).trajectory_point_size(); ++k)
            {
                cv::Point2d pt;
                pt.x = obs.trajectory(j).trajectory_point(k).path_point().x();
                pt.y = obs.trajectory(j).trajectory_point(k).path_point().y();
                ob_tra_pts.push_back(pt);
            }

            obstacles_trajectory_points.push_back(ob_tra_pts);
        }
    }

    AINFO << "prediction";

    // chassis

    double steering_angle;
    steering_angle =
            -chassis_.steering_percentage() / 100.0 * max_steering_wheel_angle_;

    chassis_data.set_x(chassis_.speed_mps());
    chassis_data.set_y(steering_angle);

    AINFO << "chassis";

    // control
    std::vector<float> can_msg;

    if (can_msg.size() < 2)
    {
        can_msg.push_back(-1000);
        can_msg.push_back(-1000);
    }
    // 角度,acc
    control_data_.set_x(can_msg[0]);
    control_data_.set_y(can_msg[1]);

    AINFO << "control";

    return 0;
}

int viz3d_component::drive_mode_update(bool key_r)
{
    drive_mode_changed_ = false;

    {
        if (drive_mode_ == apollo::canbus::Chassis::DrivingMode::
                                   Chassis_DrivingMode_COMPLETE_AUTO_DRIVE)
        {
            drive_mode_ = apollo::canbus::Chassis::DrivingMode::
                    Chassis_DrivingMode_COMPLETE_MANUAL;
            AINFO << "r key is be pressed, and system will stop";
        }
        else
        {
            drive_mode_ = apollo::canbus::Chassis::DrivingMode::
                    Chassis_DrivingMode_COMPLETE_AUTO_DRIVE;
            AINFO << "r key is be pressed, and system will start";
        }

        drive_mode_changed_ = true;
    }

    // publish drive mode, subscribed by chassis
    if (drive_mode_changed_)
    {
        apollo::canbus::Chassis chassis_drive_mode;
        chassis_drive_mode.set_driving_mode(drive_mode_);
        drive_mode_writer_->Write(chassis_drive_mode);
    }
    return 0;
}

int viz3d_component::debug_mode_update()
{
    bool debug_mode_changed_ = false;
    {
        if (debug_mode.pause_debug.enabled)
        {
            debug_mode.pause_debug.enabled = false;

            AINFO << "p key is be pressed, and system will continue";
        }
        else
        {
            debug_mode.pause_debug.enabled = true;
            AINFO << "p key is be pressed, and system will pause";
        }

        if(debug_mode.pause_debug.enabled)
        {
            debug_msg_.set_debug_mode(cyber::proto::DebugMode::pause);
        }
        else
        {
            debug_msg_.set_debug_mode( cyber::proto::DebugMode::none);
        }
        debug_mode_changed_ = true;
    }

    if (debug_mode_changed_)
    {
        debug_mode_writer_->Write(debug_msg_);
    }

    return 0;
}

int viz3d_component::process()
{

    cvt_local_polygon_to_global(&veh_global_polygon, &veh_local_polygon,
                                &veh_global_pose);


    return 0;
}

}  // namespace apollo