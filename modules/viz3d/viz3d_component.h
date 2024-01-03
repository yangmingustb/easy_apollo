#pragma once

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/time/time.h"

#include "modules/canbus/proto/chassis.pb.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"

#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/common/util/util.h"
#include "modules/common/util/debug_mode.h"

// #include "modules/control/common/dependency_injector.h"
// #include "modules/control/controller/controller_agent.h"
// #include "modules/control/proto/preprocessor.pb.h"
// #include "modules/control/submodules/preprocessor_submodule.h"

#include "modules/prediction/proto/prediction_obstacle.pb.h"


#include "modules/dreamview/backend/map/map_service.h"

#include "cyber/proto/run_mode_conf.pb.h"
#include "cyber/proto/record.pb.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/viz2d/viz_subscribe.h"
#include "modules/viz2d/viz2d_traffic_light.h"

#include "modules/planning/common/path/discretized_path.h"

#include "modules/common/math/polygon_base.h"
#include "modules/common/status/system_state.h"

#include "modules/planning/reference_line/reference_line.h"

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


namespace apollo
{

// 使用pcl坐标, east-north-up
// 并且，所有数据都需要转换成vehicle 坐标系，vehicle 坐标系是right-front-up
class viz3d_component
{
private:
    // reader
    std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>>
            chassis_reader_;

    std::shared_ptr<
            apollo::cyber::Reader<apollo::prediction::PredictionObstacles>>
            prediction_reader_;

    std::shared_ptr<cyber::Reader<apollo::perception::PerceptionObstacles>>
            perception_reader_;

    std::shared_ptr<
            apollo::cyber::Reader<apollo::localization::LocalizationEstimate>>
            localization_reader_;

    std::shared_ptr<apollo::cyber::Reader<apollo::control::ControlCommand>>
            control_cmd_reader_;

    std::shared_ptr<cyber::Reader<apollo::planning::ADCTrajectory>>
            trajectory_reader_;

    std::shared_ptr<cyber::Reader<apollo::routing::RoutingResponse>>
            routing_response_reader_;

    std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
            traffic_light_reader_;

    std::shared_ptr<apollo::cyber::Reader<apollo::cyber::proto::PlayInfo>>
            play_state_reader_;

    // read from record
    std::shared_ptr<
            apollo::cyber::Reader<apollo::perception::PerceptionObstacles>>
            hmi_perception_reader_;

    // writer

    std::shared_ptr<
            apollo::cyber::Writer<apollo::perception::PerceptionObstacles>>
            obs_writer_;

    // 发送drive mode
    std::shared_ptr<apollo::cyber::Writer<apollo::canbus::Chassis>>
            drive_mode_writer_;

    std::shared_ptr<apollo::cyber::Writer<apollo::cyber::proto::DebugMsg>>
            debug_mode_writer_;

    std::shared_ptr<apollo::cyber::Writer<perception::TrafficLightDetection>>
            traffic_light_detection_writer_ = nullptr;

    std::shared_ptr<apollo::cyber::Writer<planning::LaneBorrowManual>>
            lane_borrow_manual_writer_ = nullptr;

    std::mutex mutex_;

    // node
    std::shared_ptr<apollo::cyber::Node> node_;

    // external data
    apollo::canbus::Chassis chassis_;
    apollo::control::ControlCommand control_cmd_;
    apollo::prediction::PredictionObstacles prediction_;

    apollo::perception::PerceptionObstacles perception_;
    apollo::perception::PerceptionObstacles hmi_perception_;

    apollo::localization::LocalizationEstimate localization_;
    apollo::planning::ADCTrajectory trajectory_;
    apollo::routing::RoutingResponse route_;

    // manual lane borrow
    planning::LaneBorrowManual  lane_borrow_manual_;
    bool lane_borrow_manual_changed_;

    // copy data
    viz_subscribe viz_subscribe_;

    std::unique_ptr<apollo::dreamview::MapService> map_service;

    // munual traffic light
    perception::TrafficLightDetection manual_traffic_light_;

    ManualTrafficLight manual_traffic_light_generator_;

    perception::TrafficLight::Color manual_traffic_light_color_;

    bool manual_traffic_light_color_changed_ = false;
    

    /*******************************/
    // hmi related
    // 自动驾驶模式消息
    apollo::canbus::Chassis::DrivingMode drive_mode_;
    bool drive_mode_changed_ = false;

    apollo::cyber::proto::DebugMsg debug_msg_;

    // -1, none
    // 0, 静态
    // 1， 动态
    // 根据值去添加不同类型的障碍物
    // 操作：先按键盘，如s，表示静态障碍物，再鼠标点击，表示obs的位置
    int virtual_obs_speed_type_;

    perception::PerceptionObstacle::Type virtual_obs_type_;

    apollo::cyber::proto::PlayInfo play_info_;

    // ***********************
    // internal  data

    apollo::common::VehicleConfig vehicle_config;

    Polygon2D veh_local_polygon;
    Polygon2D veh_global_polygon;

    Pose2D veh_global_pose;
    Pose2D hmap_base_pose;

    double wheel_base;
    double front_wheel_angle;

    apollo::RunningTimeDebug debug_mode;
    int key_value_ = -1;

    std::vector<apollo::hdmap::MapPathPoint> routing_points_;
    std::vector<apollo::hdmap::Path> route_path_list_;
    bool is_new_route_;

    apollo::localization::Pose veh_pose_;

    apollo::common::math::Vec2d chassis_data;
    apollo::common::math::Vec2d control_data_;
    apollo::common::math::Vec2d history_control_data_;

    std::vector<std::vector<Position2D>> obstacles_trajectory_points;

    // todo: draw all ref lines
    std::list<planning::ReferenceLine> ref_lines_;
    SystemState module_state_;

    apollo::cyber::proto::RunMode run_mode_;
    // *******************************

    // safe buffer
    double decision_safe_buffer_;

    double path_bound_safe_buffer_;

    pcl::visualization::PCLVisualizer *window_;

public:
    viz3d_component(/* args */);
    ~viz3d_component(){};

    int init();

    int process(double max_steering_wheel_angle_);

    int data_tanslate(double max_steering_wheel_angle_,
                      apollo::planning::DiscretizedPath& lat_path);

    int close();

    int reset_viz();

    void set_camera();

    void refresh_in_per_frame();

    bool pcl_viz_stopped();
};
}  // namespace apollo