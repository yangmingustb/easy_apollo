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
#include "modules/control/common/dependency_injector.h"
#include "modules/control/controller/controller_agent.h"
#include "modules/control/proto/preprocessor.pb.h"
#include "modules/control/submodules/preprocessor_submodule.h"

#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/planning/common/path/discretized_path.h"
#include "modules/planning/reference_line/reference_line.h"


#include "cyber/proto/run_mode_conf.pb.h"
#include "modules/dreamview/backend/map/map_service.h"

#include "modules/viz2d/viz2d_map_elements.h"
#include "modules/viz2d/viz2d_geometry.h"



namespace apollo
{

// 注意，所有的元素最好提前分配好，然后每一帧刷新内部数据即可。
// 如果每一帧都分配新的内存，要记得在下一帧开始的时候，删除数据

class viz3d_component
{
public:
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

    // writer

    std::shared_ptr<
            apollo::cyber::Writer<apollo::perception::PerceptionObstacles>>
            obs_writer_;

    // 发送drive mode
    std::shared_ptr<apollo::cyber::Writer<apollo::canbus::Chassis>>
            drive_mode_writer_;

                std::shared_ptr<apollo::cyber::Writer<apollo::cyber::proto::DebugMsg>>
            debug_mode_writer_;

    std::mutex mutex_;

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


    apollo::canbus::Chassis::DrivingMode drive_mode_;
    bool drive_mode_changed_ = false;

    apollo::cyber::proto::DebugMsg debug_msg_;

    apollo::planning::DiscretizedPath lateral_path_;

    apollo::common::VehicleConfig vehicle_config;

    double max_steering_wheel_angle_;

    Polygon2D veh_local_polygon;
    Polygon2D veh_global_polygon;

    Pose2D veh_global_pose;
    Pose2D hmap_base_pose;

    double wheel_base;
    double front_wheel_angle;

    apollo::RunningTimeDebug debug_mode;
    int key_value = -1;

    std::vector<cv::Vec<double, 10>> trajectory_points;
    std::vector<std::vector<cv::Point2d>> obstacles_trajectory_points;
    cv::Vec3d cv_loc;
    apollo::localization::Pose veh_pose_;

    apollo::common::math::Vec2d chassis_data;
    apollo::common::math::Vec2d control_data_;
    apollo::common::math::Vec2d history_control_data_;

    // todo: draw all ref lines
    std::list<planning::ReferenceLine> ref_lines_;
    SystemState module_state_;

    apollo::cyber::proto::RunMode run_mode_;

//     cv::viz::WWidgetMerger map_merger_;
    // *******************************

public:
    viz3d_component(/* args */);
    ~viz3d_component(){};

    int init();

    int process();

    int data_tanslate();

    int close();

   int drive_mode_update(bool key_r);
   int debug_mode_update();

};
}  // namespace apollo