#pragma once

#include <chrono>
#include <iostream>
#include <memory>
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/planning_base.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/viz2d/viz_window.h"
#include "modules/viz2d/opencv_viz.h"
#include "modules/common/math/polygon_base.h"
#include "cyber/proto/run_mode_conf.pb.h"

namespace apollo
{
namespace planning
{
#define debug_planning_component (0)

// 计算预测、规划
class PlanningComponent
{
public:
    PlanningComponent(){};

    int init(std::string config_dir);

    //
    int process(std::shared_ptr<apollo::routing::RoutingResponse>
                        &routing_response_ptr);

    int update_planning(std::shared_ptr<apollo::canbus::Chassis> &chassis_ptr,
                     std::shared_ptr<apollo::localization::LocalizationEstimate>
                             &localization_estimate_ptr,
                     std::shared_ptr<apollo::prediction::PredictionObstacles>
                             &prediction_obstacles_ptr,
                     std::shared_ptr<apollo::routing::RoutingResponse>
                             &routing_response_ptr);

    apollo::planning::DiscretizedPath get_current_frame_planned_path();

    // 释放
    int release();

    const apollo::planning::ReferenceLineInfo *get_ref_line_info()
    {
        if (planning_ == nullptr)
        {
            return nullptr;
        }

        if (planning_->injector_ == nullptr)
        {
            return nullptr;
        }

        const apollo::planning::FrameHistory *history =
                planning_->injector_->frame_history();

        if (history == nullptr)
        {
            return nullptr;
        }

        const apollo::planning::Frame *previous_frame = history->Latest();

        if (previous_frame == nullptr)
        {
            return nullptr;
        }

        const apollo::planning::ReferenceLineInfo *ref_line =
                previous_frame->DriveReferenceLineInfo();

        return ref_line;
    }

    int debug_path_init();

    int debug_path_in_viz();

    bool check_new_routing_request(double timestamp)
    {
        bool new_request = false;
        if (!routing_request_.has_header())
        {
            return false;
        }

        if (routing_request_.waypoint_size() < 2)
        {
            return false;
        }

        for (const auto &waypoint : routing_request_.waypoint())
        {
            if (!waypoint.has_id() || !waypoint.has_s())
            {
                return false;
            }
        }

        double  delta_time = 100000.0;

        if(routing_request_.header().has_timestamp_sec())
        {
            delta_time = timestamp - routing_request_.header().timestamp_sec();
        }

        // 路由请求已经超过2分钟，认为无效
        if (delta_time > 120.0)
        {
            return false;
        }

        const routing::RoutingRequest &history_request =
                local_view_.routing->routing_request();

        if (common::util::IsProtoEqual(history_request, routing_request_))
        {
            return false;
        }

        return true;
    }

    bool is_new_routing_request() { return is_new_routing_request_; }

    const std::shared_ptr<routing::RoutingRequest> &get_latest_routing_request()
    {
        return routing_request_copy_;
    }

    int update_cyber_frame_data();

    const std::shared_ptr<apollo::perception::PerceptionObstacles> &
    get_const_perception() const;

    const std::shared_ptr<apollo::localization::LocalizationEstimate> &
    get_const_localization() const;

    std::shared_ptr<apollo::prediction::PredictionObstacles> &
    get_mutable_prediction();

    const ADCTrajectory &get_latest_trajectory() const;

    const std::shared_ptr<ADCTrajectory> &get_latest_trajectory_ptr() const;

protected:
    bool IsValidTrajectory(apollo::planning::ADCTrajectory &trajectory);

    // protected:
public:
    std::unique_ptr<apollo::planning::PlanningBase> planning_ = nullptr;
    std::map<apollo::planning::TrafficRuleConfig::RuleId, bool> rule_enabled_;

    // output
    ADCTrajectory adc_trajectory_;
    std::shared_ptr<ADCTrajectory> adc_trajectory_ptr;

    apollo::planning::LocalView local_view_;
    apollo::planning::PlanningConfig config_;
    std::shared_ptr<apollo::planning::DependencyInjector> injector_;

    // reader

    std::shared_ptr<apollo::cyber::Reader<perception::TrafficLightDetection>>
            traffic_light_reader_;

    std::shared_ptr<apollo::cyber::Reader<localization::LocalizationEstimate>>
            localization_reader_;

    std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>>
            chassis_reader_;

    std::shared_ptr<
            apollo::cyber::Reader<apollo::perception::PerceptionObstacles>>
            perception_reader_;
    std::shared_ptr<
            apollo::cyber::Reader<apollo::perception::PerceptionObstacles>>
            hmi_perception_reader_;

    std::shared_ptr<
            apollo::cyber::Reader<apollo::cyber::proto::DebugMsg>>
            debug_msg_reader_;

    std::shared_ptr<apollo::cyber::Reader<routing::RoutingRequest>>
            routing_request_reader_;

    std::shared_ptr<apollo::cyber::Reader<planning::LaneBorrowManual>>
            lane_borrow_manual_reader_;

    // writer
    std::shared_ptr<cyber::Writer<planning::ADCTrajectory>> traj_writer_;
    std::shared_ptr<::apollo::cyber::Writer<apollo::routing::RoutingResponse>>
            route_writer_;
    std::shared_ptr<cyber::Writer<prediction::PredictionObstacles>>
            prediction_writer_;

    std::mutex mutex_;


    // routing::RoutingResponse *routing_;
    std::shared_ptr<apollo::prediction::PredictionObstacles> prediction_;

    // input data
    apollo::canbus::Chassis chassis_;

    std::shared_ptr<apollo::canbus::Chassis> ptr_chassis_;

    // 不会每一帧都刷新，如果没有刷新，那么历史数据会记录在里面
    apollo::perception::PerceptionObstacles perception_;
    apollo::perception::PerceptionObstacles hmi_perception_;

    std::shared_ptr<apollo::perception::PerceptionObstacles>
            perception_obstacles_;

    apollo::localization::LocalizationEstimate localization_;

    std::shared_ptr<apollo::localization::LocalizationEstimate>
            ptr_localization_;

    perception::TrafficLightDetection traffic_light_;

    routing::RoutingRequest     routing_request_;

    std::shared_ptr<routing::RoutingRequest> routing_request_copy_;

    bool is_new_routing_request_;

    planning::LaneBorrowManual lane_borrow_manual_;

    // node

    std::shared_ptr<apollo::cyber::Node> node_;

    // debug相关消息
    apollo::cyber::proto::DebugMsg debug_msg_;

    // debug related
    apollo::viz2d_image *debug_window2d_ = NULL;

    Polygon2D veh_local_polygon;
    Polygon2D veh_global_polygon;

    Pose2D veh_global_pose;

    const std::list<apollo::planning::ReferenceLine> *ref_lines;
};
}  // namespace planning
}

