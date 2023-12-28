
#include "cyber/time/clock.h"
#include "modules/common/util/message_util.h"
#include "modules/planning/on_lane_planning.h"

#include "modules/common/adapters/adapter_gflags.h"

#include "modules/viz2d/viz2d_path.h"

#include "modules/viz2d/viz2d_geometry.h"
#include "modules/viz2d/viz2d_map_elements.h"
#include "modules/planning/planning_component.h"
#include "modules/viz2d/viz2d_perception.h"

namespace apollo
{
namespace planning
{
using namespace apollo::canbus;
using namespace apollo::cyber;
using namespace apollo::prediction;
using namespace apollo::localization;

#define debug_obstacle_prediction (0)

// apollo的规划，需要先确定场景，再确定阶段，再调用计算任务，最后输出轨迹。所以，
// 需要对不同的场景、阶段、任务，都配置好相应的参数，有些参数在
// .txt文件中，有些参数在google_flag.h定义中;

int PlanningComponent::init(std::string config_dir)
{
    node_ = apollo::cyber::CreateNode(apollo::cyber::binary::GetName());

    chassis_reader_ = node_->CreateReader<canbus::Chassis>(
            FLAGS_chassis_topic,
            [this](const std::shared_ptr<canbus::Chassis> &chassis)
            {
                ADEBUG << "Received chassis data: run chassis callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                chassis_.CopyFrom(*chassis);
            });

    localization_reader_ = node_->CreateReader<
            localization::LocalizationEstimate>(
            FLAGS_localization_topic,
            [this](const std::shared_ptr<localization::LocalizationEstimate>
                           &localization)
            {
                ADEBUG << "Received chassis data: run chassis callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                localization_.CopyFrom(*localization);
            });

    perception_reader_ = node_->CreateReader<perception::PerceptionObstacles>(
            FLAGS_perception_obstacle_topic,
            [this](const std::shared_ptr<perception::PerceptionObstacles>
                           &perception_obstacles)
            {
                ADEBUG << "Received perception data: run perception callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                perception_.CopyFrom(*perception_obstacles);
            });

    hmi_perception_reader_ = node_->CreateReader<
            perception::PerceptionObstacles>(
            FLAGS_hmi_obstacle_topic,
            [this](const std::shared_ptr<perception::PerceptionObstacles>
                           &perception_obstacles)
            {
                ADEBUG << "Received perception data: run perception callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                hmi_perception_.CopyFrom(*perception_obstacles);
            });
    debug_msg_reader_ = node_->CreateReader<cyber::proto::DebugMsg>(
            FLAGS_debug_planning_msg,
            [this](const std::shared_ptr<cyber::proto::DebugMsg> &debug)
            {
                ADEBUG << "Received perception data: run perception callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                debug_msg_.CopyFrom(*debug);
            });

    traffic_light_reader_ = node_->CreateReader<
            perception::TrafficLightDetection>(
            FLAGS_traffic_light_detection_topic,
            [this](const std::shared_ptr<perception::TrafficLightDetection>
                           &traffic_light)
            {
                ADEBUG << "Received traffic light data: run traffic light "
                          "callback.";

                std::lock_guard<std::mutex> lock(mutex_);
                traffic_light_.CopyFrom(*traffic_light);
            });

    routing_request_reader_ = node_->CreateReader<
            routing::RoutingRequest>(
            FLAGS_routing_request_topic,
            [this](const std::shared_ptr<routing::RoutingRequest>
                           &routing_request)
            {
                ADEBUG << "Received routing request.";

                std::lock_guard<std::mutex> lock(mutex_);
                routing_request_.CopyFrom(*routing_request);
            });

    lane_borrow_manual_reader_ = node_->CreateReader<
            planning::LaneBorrowManual>(
            "lane_borrow_manual",
            [this](const std::shared_ptr<planning::LaneBorrowManual>
                           &lane_borrow)
            {
                ADEBUG << "Received lane borrow request.";

                std::lock_guard<std::mutex> lock(mutex_);
                lane_borrow_manual_.CopyFrom(*lane_borrow);
            });

    traj_writer_ = node_->CreateWriter<planning::ADCTrajectory>(
            FLAGS_planning_trajectory_topic);

    prediction_writer_ = node_->CreateWriter<prediction::PredictionObstacles>(
            FLAGS_prediction_topic);

    route_writer_ = node_->CreateWriter<routing::RoutingResponse>(
            FLAGS_routing_response_topic);

    //
    std::string planning_config_file =
            config_dir + "/planning/conf/planning_config.pb.txt";

    FLAGS_traffic_rule_config_filename =
            config_dir + "/planning/conf/traffic_rule_config.pb.txt";

    // FLAGS_smoother_config_filename =
    //         config_dir + "/conf/planning/qp_spline_smoother_config.pb.txt";

    // FLAGS_smoother_config_filename =
    //         config_dir +
    //         "/conf/planning/discrete_points_smoother_config.pb.txt";

    FLAGS_align_prediction_time = false;
    FLAGS_enable_reference_line_provider_thread = false;
    // FLAGS_enable_trajectory_check is temporarily disabled, otherwise
    // EMPlanner and LatticePlanner can't pass the unit test.
    FLAGS_enable_trajectory_check = true;

    // planning 最大速度
    // acc、dec在pb.txt配置

    // 设置借道相关参数

    FLAGS_long_term_blocking_obstacle_cycle_threshold = 1;

    FLAGS_enable_rss_info = false;

    // 规划周期，1 / 0.1秒
    FLAGS_planning_loop_rate = 10;

    // ================================
    // 调整弯道速度:
    // 速度优化时调整weight
    // kappa_penalty_weight
    // 调整向心加速度值，默认为2. 值越低，过弯速度就越小。
    // max_centric_acceleration_limit

    //=========================
    // 安全性限速
    // static_obs_nudge_speed_ratio: 0.6
    // dynamic_obs_nudge_speed_ratio: 0.8

    // lane change
    // FLAGS_enable_smarter_lane_change = true;
    // FLAGS_lane_change_prepare_length = 20.0;
    // 最小准备距离
    FLAGS_min_lane_change_prepare_length = 2.0;

    // obs avoid
    // FLAGS_static_obstacle_speed_threshold = 2.5;

    std::string planning_conf_dir = config_dir + "/planning/conf/";

    injector_ = std::make_shared<DependencyInjector>();

    if (FLAGS_use_navigation_mode)
    {
        // TODO(all)
        // planning_ = std::unique_ptr<PlanningBase>(new NaviPlanning());
    }
    else
    {
        planning_ =
                std::unique_ptr<PlanningBase>(new OnLanePlanning(injector_));
    }

    // ACHECK(FeedTestData()) << "Failed to feed test data";

    // 读取规划配置文件
    ACHECK(cyber::common::GetProtoFromFile(planning_config_file, &config_))
            << "failed to load planning config file " << planning_config_file;

    // 初始化配置文件
    ACHECK(planning_->Init(config_).ok()) << "Failed to init planning module";

    // 读取上一次的规划结果
#if 0
    if (!FLAGS_test_previous_planning_file.empty())
    {
        const auto prev_planning_file =
                FLAGS_test_data_dir + "/" + FLAGS_test_previous_planning_file;
        ADCTrajectory prev_planning;
        ACHECK(cyber::common::GetProtoFromFile(
                prev_planning_file, &prev_planning));
        planning_->last_publishable_trajectory_.reset(
                new PublishableTrajectory(prev_planning));
    }
#endif

    for (auto &config : *(planning_->traffic_rule_configs_.mutable_config()))
    {
        auto iter = rule_enabled_.find(config.rule_id());
        if (iter != rule_enabled_.end())
        {
            config.set_enabled(iter->second);
        }
    }

    AINFO << "init finish";

    adc_trajectory_ptr = std::make_shared<planning::ADCTrajectory>();

    hmi_perception_.Clear();
    perception_.Clear();

    perception_obstacles_ =
            std::make_shared<apollo::perception::PerceptionObstacles>();

    prediction_ = std::make_shared<apollo::prediction::PredictionObstacles>();

    ptr_chassis_ = std::make_shared<apollo::canbus::Chassis>();

    ptr_localization_ =
            std::make_shared<apollo::localization::LocalizationEstimate>();

    routing_request_copy_ = std::make_shared<apollo::routing::RoutingRequest>();

    lane_borrow_manual_.Clear();

    debug_path_init();

    // set initial vehicle state by cmd
    // need to sleep, because advertised channel is not ready immediately
    // simple test shows a short delay of 80 ms or so
    AINFO << "planning resetting vehicle state, sleeping for 1000 ms ...";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return 0;
}

bool PlanningComponent::IsValidTrajectory(apollo::planning::ADCTrajectory &trajectory)
{
    if (trajectory.trajectory_point_size() == 0)
    {
        AERROR << "trajectory point size is 0";
        return false;
    }

    for (int i = 0; i < trajectory.trajectory_point_size(); ++i)
    {
        const auto &point = trajectory.trajectory_point(i);

        // check acc
        const double kMaxAccelThreshold =
                FLAGS_longitudinal_acceleration_upper_bound;
        const double kMinAccelThreshold =
                FLAGS_longitudinal_acceleration_lower_bound;
        if (point.a() > kMaxAccelThreshold || point.a() < kMinAccelThreshold)
        {
            AERROR << "Invalid trajectory point because accel out of range: "
                   << point.DebugString();

            return false;
        }

        if (!point.has_path_point())
        {
            AERROR << "Invalid trajectory point because NO path_point in "
                      "trajectory_point: "
                   << point.DebugString();

            return false;
        }

        // check path is forward
        if (i > 0)
        {
            const double kPathSEpsilon = 1e-3;
            const auto &last_point = trajectory.trajectory_point(i - 1);
            if (point.path_point().s() + kPathSEpsilon <
                last_point.path_point().s())
            {
                AERROR << "Invalid trajectory point because s value error. "
                          "last point: "
                       << last_point.DebugString()
                       << ", curr point: " << point.DebugString();

                return false;
            }
        }
    }
    return true;
}

int PlanningComponent::update_cyber_frame_data()
{
    auto prepross_start_time = std::chrono::system_clock::now();

    //-----------------------------------------------------------
    // 需要对齐规划、预测的时间戳，因为速度规划使用到了relative time
    //----------------------------------------------------

    {
        std::lock_guard<std::mutex> lock(mutex_);

        ptr_chassis_->Clear();
        ptr_chassis_->CopyFrom(chassis_);

        perception_obstacles_->Clear();
        // 融合感知数据
        if (hmi_perception_.has_header())
        {
            perception_obstacles_->CopyFrom(hmi_perception_);

            for (int i = 0; i < perception_.perception_obstacle_size(); i++)
            {
                apollo::perception::PerceptionObstacle *obs =
                        perception_obstacles_->add_perception_obstacle();

                obs->CopyFrom(perception_.perception_obstacle().at(i));
            }
        }
        else if (perception_.has_header())
        {
            perception_obstacles_->CopyFrom(perception_);

            for (int i = 0; i < hmi_perception_.perception_obstacle_size(); i++)
            {
                apollo::perception::PerceptionObstacle *obs =
                        perception_obstacles_->add_perception_obstacle();

                obs->CopyFrom(hmi_perception_.perception_obstacle().at(i));
            }
        }

        // AINFO << "hmi_perception_ size: "
        //       << hmi_perception_.perception_obstacle_size();
        // AINFO << "perception size: " <<
        // perception_.perception_obstacle_size();
        AINFO << "total perception size: "
              << perception_obstacles_->perception_obstacle_size();
        // AINFO << "perception time: "
        //       << perception_obstacles_->header().DebugString();

        ptr_localization_->Clear();
        ptr_localization_->CopyFrom(localization_);

        local_view_.traffic_light =
                std::make_shared<perception::TrafficLightDetection>(
                        traffic_light_);

        // update lane borrow manual
        auto *mutable_path_decider_status =
                injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_lane_borrow_decider();

        planning::LaneBorrowManual *lane_borrow_manual =
                mutable_path_decider_status->mutable_lane_borrow_by_human();

        lane_borrow_manual->CopyFrom(lane_borrow_manual_);
    }

    return 0;
}

int PlanningComponent::process(
        std::shared_ptr<apollo::routing::RoutingResponse> &routing_response_ptr)
{
    if (routing_response_ptr == nullptr)
    {
        return -1;
    }

    if (routing_response_ptr->road_size() <= 0)
    {
        return -2;
    }

    if (debug_msg_.debug_mode() != cyber::proto::DebugMode::none)
    {
        AINFO << "debug mode:pause";
        return 0;
    }

    // 不使用定位的acc,angular, 设备精度不足，影响运动规划模块
    apollo::localization::Pose *pose = ptr_localization_->mutable_pose();

    pose->mutable_linear_acceleration()->set_x(0);
    pose->mutable_linear_acceleration()->set_y(0);
    pose->mutable_linear_acceleration()->set_z(0);

    pose->mutable_linear_acceleration_vrf()->set_x(0);
    pose->mutable_linear_acceleration_vrf()->set_y(0);
    pose->mutable_linear_acceleration_vrf()->set_z(0);

    pose->mutable_angular_velocity()->set_x(0);
    pose->mutable_angular_velocity()->set_y(0);
    pose->mutable_angular_velocity()->set_z(0);

    pose->mutable_angular_velocity_vrf()->set_x(0);
    pose->mutable_angular_velocity_vrf()->set_y(0);
    pose->mutable_angular_velocity_vrf()->set_z(0);

    auto end_time2 = std::chrono::system_clock::now();

    // planning
    adc_trajectory_.Clear();

    // AINFO << ptr_chassis_->DebugString();
    int rval = update_planning(ptr_chassis_, ptr_localization_, prediction_,
                               routing_response_ptr);

    auto end_time4 = std::chrono::system_clock::now();
    std::chrono::duration<double> planning_diff = end_time4 - end_time2;

    // std::cout << "route road seg size "
    //           << plan_data->_ptr_routing_response->road_size() <<
    //           "\n";

    // const apollo::planning::ReferenceLineInfo *ref_line = get_ref_line_info();

    // if (ref_line != nullptr)
    // {
    //     plt_draw_st_data(ref_line->speed_data(),
    //                      ref_line->path_decision().obstacles());
    // }

    // publish prediction
    prediction_writer_->Write(prediction_);

    auto post_process_time_end = std::chrono::system_clock::now();
    std::chrono::duration<double> post_process_diff =
            post_process_time_end - end_time4;

    AINFO << "print_planning_time:"
          << "(" << planning_diff.count() * 1000<< ","
          << ")";
    AINFO << "print_post_process_time:"
          << "(" << post_process_diff.count() * 1000 << ","
          << ")";

    return 0;
}

int PlanningComponent::update_planning(
        std::shared_ptr<apollo::canbus::Chassis> &chassis_ptr,
        std::shared_ptr<apollo::localization::LocalizationEstimate>
                &localization_estimate_ptr,
        std::shared_ptr<apollo::prediction::PredictionObstacles>
                &prediction_obstacles_ptr,
        std::shared_ptr<apollo::routing::RoutingResponse> &routing_response_ptr)
{
    if (routing_response_ptr == nullptr)
    {
        return 0;
    }

    // publish routing
    {
        std::lock_guard<std::mutex> lock(mutex_);
        bool current_frame_publish_route = false;
        if (!local_view_.routing ||
            hdmap::PncMap::IsNewRouting(*local_view_.routing,
                                        *routing_response_ptr))
        {
            route_writer_->Write(routing_response_ptr);

            local_view_.routing = std::make_shared<routing::RoutingResponse>(
                    (*routing_response_ptr));

            current_frame_publish_route = true;
            AINFO << "publish route";
        }

        // todo:持续发送route一段时间,10 s
        double k_route_publish_time = 3.0;
        const double timestamp = Clock::NowInSeconds();
        if (!current_frame_publish_route)
        {
            if (timestamp - local_view_.routing->header().timestamp_sec() <
                k_route_publish_time)
            {
                route_writer_->Write(routing_response_ptr);

                AINFO << "publish route";
            }
        }

        is_new_routing_request_ = check_new_routing_request(timestamp);
        if (is_new_routing_request_)
        {
            routing_request_copy_->Clear();
            routing_request_copy_->CopyFrom(routing_request_);
            AINFO << "is_new_routing_request_ " << is_new_routing_request_;
        }


    }



    local_view_.prediction_obstacles = prediction_obstacles_ptr;
    local_view_.chassis = chassis_ptr;
    local_view_.localization_estimate = localization_estimate_ptr;

    if (prediction_obstacles_ptr == nullptr || chassis_ptr == nullptr ||
        localization_estimate_ptr == nullptr)
    {
        return 0;
    }

    if (!chassis_ptr->has_header())
    {
        return 0;
    }
    if (!prediction_obstacles_ptr->has_header())
    {
        return 0;
    }
    if (!localization_estimate_ptr->has_header())
    {
        return 0;
    }
    if (!routing_response_ptr->has_header())
    {
        return 0;
    }

    // AINFO << prediction_.DebugString();
    // AINFO << chassis_.DebugString();
    // AINFO << localization_.DebugString();
    // AINFO << local_view_.routing->road_size();

    //
    planning_->RunOnce(local_view_, &adc_trajectory_);

    if (!IsValidTrajectory(adc_trajectory_))
    {
        AERROR << "Fail to pass trajectory check.";
        return -1;
    }
#if debug_planning_component
    else
    {
        AINFO << "traj is valid";
    }
#endif

    // publish
    auto start_time = adc_trajectory_.header().timestamp_sec();
    apollo::common::util::FillHeader("planning", &adc_trajectory_);

#if debug_planning_component
    AINFO << "cyber traj sequence: " << adc_trajectory_.header().sequence_num();
#endif

    // modify trajectory relative time due to the timestamp change in header
    const double dt = start_time - adc_trajectory_.header().timestamp_sec();
    for (auto &p : *adc_trajectory_.mutable_trajectory_point())
    {
        p.set_relative_time(p.relative_time() + dt);
    }

    // record in history
    auto *history = injector_->history();
    history->Add(adc_trajectory_);

    auto publish_time_start = std::chrono::system_clock::now();

    traj_writer_->Write(adc_trajectory_);

    auto publish_time_end = std::chrono::system_clock::now();
    std::chrono::duration<double> publish_time =
            publish_time_end - publish_time_start;

    AINFO << "print_traj_publish_time:"
          << "(" << publish_time.count() * 1000 << ","
          << ")";

    debug_path_in_viz();

#if debug_planning_component

    AINFO << adc_trajectory_.decision().DebugString();

    for (size_t i = 0; i < adc_trajectory_.trajectory_point_size(); i++)
    {
        AINFO << adc_trajectory_.trajectory_point(i).DebugString();

        if (adc_trajectory_.trajectory_point(i).path_point().s() > 5.0)
        {
            break;
        }
    }

    // AINFO << adc_trajectory_.DebugString();
#endif
    return 0;
}

apollo::planning::DiscretizedPath PlanningComponent::get_current_frame_planned_path()
{
    apollo::planning::DiscretizedPath path;
    if (nullptr == injector_)
    {
        return path;
    }

    if (injector_->frame_history() != nullptr)
    {
        const auto &history_frame = injector_->frame_history()->Latest();

        if (history_frame != nullptr)
        {
            const DiscretizedPath &history_path =
                    history_frame->current_frame_planned_path();

            return history_path;
        }
    }

    return path;
}

int PlanningComponent::release()
{
    if (FLAGS_debug_path)
    {
        viz2d_release(debug_window2d_);
    }

    return 0;
}

int PlanningComponent::debug_path_in_viz()
{
    if (!FLAGS_debug_path)
    {
        return 0;
    }
    viz2d_init_in_per_frame(debug_window2d_);

    init_path_flag();

    // localization

    veh_global_pose.pos.x = ptr_localization_->pose().position().x();
    veh_global_pose.pos.y = ptr_localization_->pose().position().y();
    veh_global_pose.theta = ptr_localization_->pose().heading();

    cvt_local_polygon_to_global(&veh_global_polygon, &veh_local_polygon,
                                &veh_global_pose);

    viz2d_draw_xy_axis(debug_window2d_);

    viz2d_draw_polygon(debug_window2d_, &veh_global_polygon, &veh_global_pose,
                                    viz2d_colors_green);

    // hdmap
    // viz2d_draw_hdmap(viz2d, &veh_global_pose, false);
    viz2d_draw_simple_hdmap(debug_window2d_, &veh_global_pose);

    // route
    // std::vector<cv::Vec2d> points_enu;

    // RoutingNoa *routing_noa_ = get_routing_component();

    // const std::vector<apollo::routing::NodeWithRange> &routing =
    //         routing_noa_->_routing.get_result_nodes();

    // nodes_2_vec(routing, points_enu);

    // viz2d_draw_route(viz2d, points_enu, &veh_global_pose, viz2d_colors_gray,
    //                 1);

    // apollo perception
    draw_obs_list(perception_obstacles_, veh_global_pose, debug_window2d_);

    // path
    if (injector_->frame_history() != nullptr)
    {
        const auto &history_frame = injector_->frame_history()->Latest();

        if (history_frame != nullptr)
        {
            // reference line
            const std::list<apollo::planning::ReferenceLineInfo>
                    &reference_line_info = history_frame->reference_line_info();

            viz2d_draw_ref_line_info(debug_window2d_, reference_line_info,
                                    &veh_global_pose);

            const DiscretizedPath &history_path =
                    history_frame->current_frame_planned_path();

            viz2d_draw_path(debug_window2d_, history_path, &veh_global_pose,
                           viz2d_colors_green, 2);

            std::string str = "target_path: ";

            str += history_path.get_path_boundary_name();

            draw_path_flag(viz2d_colors_green, debug_window2d_, str.c_str());
        }

        // auto seq_num = history_frame->SequenceNum();
        // AINFO << "seq_num " << seq_num;

        // const auto &history_frame1 =
        //         injector_->frame_history()->Find(seq_num - 1);

        // if (history_frame1 != nullptr)
        // {
        //     const DiscretizedPath &history_path =
        //             history_frame1->current_frame_planned_path();

        //     viz2d_draw_path(viz2d, history_path, &veh_global_pose,
        //                    viz2d_colors_red, 1);

        //     AINFO << "histroy 1";
        // }

        // const auto &history_frame2 =
        //         injector_->frame_history()->Find(seq_num - 2);

        // if (history_frame2 != nullptr)
        // {
        //     const DiscretizedPath &history_path =
        //             history_frame2->current_frame_planned_path();

        //     viz2d_draw_path(viz2d, history_path, &veh_global_pose,
        //                    viz2d_colors_black, 1);

        //     AINFO << "histroy 2";
        // }
        // const auto &history_frame4 =
        //         injector_->frame_history()->Find(seq_num - 4);

        // if (history_frame4 != nullptr)
        // {
        //     const DiscretizedPath &history_path =
        //             history_frame4->current_frame_planned_path();

        //     viz2d_draw_path(viz2d, history_path, &veh_global_pose,
        //                    VIZ2d_COLORS_BLUE, 1);

        //     AINFO << "histroy 2";
        // }
    }

    viz2d_show_result_in_per_frame(debug_window2d_);

    return 0;
}

int PlanningComponent::debug_path_init()
{
    if (!FLAGS_debug_path)
    {
        return 0;
    }

    main_window2d_init(viz2d_colors_white);

    debug_window2d_ = get_main_window2d();

    viz2d_init_image(debug_window2d_);

    veh_local_polygon = init_adv_box(
            apollo::common::VehicleConfigHelper::GetConfig().vehicle_param());

    return 0;
}

const std::shared_ptr<apollo::perception::PerceptionObstacles> &
PlanningComponent::get_const_perception() const
{
    return perception_obstacles_;
}

const std::shared_ptr<apollo::localization::LocalizationEstimate> &
PlanningComponent::get_const_localization() const
{
    return ptr_localization_;
}

std::shared_ptr<apollo::prediction::PredictionObstacles> &
PlanningComponent::get_mutable_prediction()
{
    return prediction_;
}

const ADCTrajectory &PlanningComponent::get_latest_trajectory() const
{
    if (injector_ == nullptr || injector_->frame_history() == nullptr)
    {
        return adc_trajectory_;
    }

    if (injector_->frame_history()->Latest() == nullptr)
    {
        return adc_trajectory_;
    }

    const auto &last_trajectory = injector_->frame_history()
                                          ->Latest()
                                          ->current_frame_planned_trajectory();

    return last_trajectory;
}

const std::shared_ptr<ADCTrajectory> &
PlanningComponent::get_latest_trajectory_ptr() const
{
    return adc_trajectory_ptr;
}

}  // namespace planning
}