
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
// #include "modules/common/latency_recorder/latency_recorder.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"

#include "modules/common/util/message_util.h"

#include "cyber/cyber.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/control_component.h"

namespace apollo
{
namespace control
{

using namespace apollo::planning;
using namespace apollo::canbus;
using namespace apollo::cyber;
using namespace apollo::localization;
using namespace apollo::control;

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::cyber::Clock;


ControlComponent::ControlComponent() {}

int ControlComponent::init(std::string config_dir)
{
    std::string control_dir = config_dir + "/control/conf/";
    
    FLAGS_control_conf_file = control_dir + "control_conf.pb.txt";
    
    FLAGS_control_common_conf_file = control_dir + "control_common_conf.pb.txt";
    FLAGS_mpc_controller_conf_file = control_dir + "mpc_controller_conf.pb.txt";
    FLAGS_lateral_controller_conf_file =
            control_dir + "lateral_controller_conf.pb.txt";
    FLAGS_longitudinal_controller_conf_file =
            control_dir + "longitudinal_controller_conf.pb.txt";
    FLAGS_calibration_table_file = control_dir + "calibration_table.pb.txt";

    FLAGS_set_steer_limit = true;

    // create talker node
    node_ = apollo::cyber::CreateNode(apollo::cyber::binary::GetName());

    injector_ = std::make_shared<apollo::control::DependencyInjector>();
    init_time_ = Clock::Now();

    AINFO << "Control module init, starting ...";

    if (0)
    {
        ACHECK(cyber::common::GetProtoFromFile(FLAGS_control_conf_file,
                                               &control_conf_))
                << "Unable to load control conf file: " +
                           FLAGS_control_conf_file;
    }
    else
    {
        ACHECK(cyber::common::GetProtoFromFile(FLAGS_mpc_controller_conf_file,
                                               &control_conf_))
                << "Unable to load control conf file: " +
                           FLAGS_control_conf_file;
    }

    // AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";
    // AINFO << "Conf file: " << ConfigFilePath() << " is loaded.";

    FLAGS_enable_gain_scheduler = control_conf_.enable_gain_scheduler();
    FLAGS_enable_maximum_steer_rate_limit = true;

    // initial controller agent when not using control submodules
    ADEBUG << "FLAGS_use_control_submodules: " << FLAGS_use_control_submodules;

    if (!FLAGS_use_control_submodules &&
        !controller_agent_.Init(injector_, &control_conf_).ok())
    {
        // set controller
        ADEBUG << "original control";
        // monitor_logger_buffer_.ERROR("Control init controller failed!
        // Stopping...");
        return false;
    }

    chassis_reader_ = node_->CreateReader<apollo::canbus::Chassis>(
            FLAGS_chassis_topic,
            [this](const std::shared_ptr<apollo::canbus::Chassis> &chassis)
            {
                ADEBUG << "Received chassis data: run chassis callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                latest_chassis_.CopyFrom(*chassis);
            });

    localization_reader_ = node_->CreateReader<
            apollo::localization::LocalizationEstimate>(
            FLAGS_localization_topic,
            [this](const std::shared_ptr<
                    apollo::localization::LocalizationEstimate> &localization)
            {
                ADEBUG << "Received chassis data: run chassis callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                latest_localization_.CopyFrom(*localization);
            });

    trajectory_reader_ = node_->CreateReader<apollo::planning::ADCTrajectory>(
            FLAGS_planning_trajectory_topic,
            [this](const std::shared_ptr<apollo::planning::ADCTrajectory>
                           &trajectory)
            {
                ADEBUG << "Received chassis data: run chassis callback.";
                std::lock_guard<std::mutex> lock(mutex_);
                latest_trajectory_.CopyFrom(*trajectory);
            });

    control_cmd_writer_ = node_->CreateWriter<apollo::control::ControlCommand>(
            FLAGS_control_command_topic);

    AINFO << "begin wait";

    // set initial vehicle state by cmd
    // need to sleep, because advertised channel is not ready immediately
    // simple test shows a short delay of 80 ms or so
    // AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // should init_vehicle first, let car enter work status, then use status msg
    // trigger control

    // AINFO << "Control default driving action is " <<
    // DrivingAction_Name(control_conf_.action());
    // pad_msg_.set_action(control_conf_.action());

    AINFO << "init finish";
    init_finish_ = true;
    return true;
}

int ControlComponent::update_cyber_rt()
{
    {
        // TODO(SHU): to avoid redundent copy
        std::lock_guard<std::mutex> lock(mutex_);
        local_view_.mutable_chassis()->CopyFrom(latest_chassis_);
        local_view_.mutable_trajectory()->CopyFrom(latest_trajectory_);
        local_view_.mutable_localization()->CopyFrom(latest_localization_);

        // 不使用定位的acc,angular, 设备精度不足，影响控制模块
        if (1)
        {
            apollo::localization::Pose *pose =
                    local_view_.mutable_localization()->mutable_pose();

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
        }
    }

    // use control submodules, 0
    if (FLAGS_use_control_submodules)
    {
        local_view_.mutable_header()->set_lidar_timestamp(
                local_view_.trajectory().header().lidar_timestamp());
        local_view_.mutable_header()->set_camera_timestamp(
                local_view_.trajectory().header().camera_timestamp());
        local_view_.mutable_header()->set_radar_timestamp(
                local_view_.trajectory().header().radar_timestamp());
        // common::util::FillHeader(FLAGS_control_local_view_topic,
        // &local_view_);

        const auto end_time = Clock::Now();

        // measure latency
        // static apollo::common::LatencyRecorder latency_recorder(
        //    FLAGS_control_local_view_topic);
        // latency_recorder.AppendLatencyRecord(
        //    local_view_.trajectory().header().lidar_timestamp(), start_time,
        //    end_time);

        local_view_writer_->Write(local_view_);
        return 0;
    }

    return 0;
}

int ControlComponent::process()
{
    const auto start_time = Clock::Now();

    // std::cout << "speed loc x: " <<
    // localization_estimate_ptr->pose().linear_velocity().x() << "\n";
    // std::cout << "speed loc y: " <<
    // localization_estimate_ptr->pose().linear_velocity().y() << "\n";

    // std::cout << "speed inj: " <<
    // injector_->vehicle_state()->linear_velocity() << "\n";

    if (control_conf_.is_control_test_mode() &&
        control_conf_.control_test_duration() > 0 &&
        (start_time - init_time_).ToSecond() >
                control_conf_.control_test_duration())
    {
        AERROR << "Control finished testing. exit";
        return -1;
    }

    control_command_.Clear();

    Status status = ProduceControlCommand(&control_command_);

    AERROR_IF(!status.ok())
            << "Failed to produce control command:" << status.error_message();

    // forward estop reason among following control frames.
    if (estop_)
    {
        control_command_.mutable_header()->mutable_status()->set_msg(
                estop_reason_);
    }

    // set header
    control_command_.mutable_header()->set_lidar_timestamp(
            local_view_.trajectory().header().lidar_timestamp());
    control_command_.mutable_header()->set_camera_timestamp(
            local_view_.trajectory().header().camera_timestamp());
    control_command_.mutable_header()->set_radar_timestamp(
            local_view_.trajectory().header().radar_timestamp());

    // apollo::common::util::FillHeader(node_->Name(), &control_command_);

    ADEBUG << control_command_.ShortDebugString();
    if (control_conf_.is_control_test_mode())
    {
        ADEBUG << "Skip publish control command in test mode";
        return 0;
    }

    const auto end_time = Clock::Now();
    const double time_diff_ms = (end_time - start_time).ToSecond() * 1e3;
    // std::cout << "total control time spend: " << time_diff_ms << " ms.\n";

    control_command_.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
    control_command_.mutable_latency_stats()->set_total_time_exceeded(
            time_diff_ms > control_conf_.control_period() * 1e3);

    status.Save(control_command_.mutable_header()->mutable_status());

    // measure latency
    // if (local_view_.trajectory().header().has_lidar_timestamp())
    // {
    //     static apollo::common::LatencyRecorder latency_recorder(
    //             FLAGS_control_command_topic);
    //     latency_recorder.AppendLatencyRecord(
    //             local_view_.trajectory().header().lidar_timestamp(),
    //             start_time, end_time);
    // }


    // left turn is positive for apollo

    return 0;
}

Status ControlComponent::ProduceControlCommand(ControlCommand *control_command)
{
    Status status = CheckInput(&local_view_);

    // Status status(ErrorCode::OK);
    // check data
    if (!status.ok())
    {
        AERROR_EVERY(100) << "Control input data failed: "
                          << status.error_message();

        control_command->mutable_engage_advice()->set_advice(
                apollo::common::EngageAdvice::DISALLOW_ENGAGE);
        control_command->mutable_engage_advice()->set_reason(
                status.error_message());
        estop_ = true;
        estop_reason_ = status.error_message();
    }
    else
    {
        // Status status_ts = CheckTimestamp(local_view_);
        Status status_ts(ErrorCode::OK);
        if (!status_ts.ok())
        {
            AERROR << "Input messages timeout";
            // estop_ = true;
            status = status_ts;
            if (local_view_.chassis().driving_mode() !=
                apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE)
            {
                control_command->mutable_engage_advice()->set_advice(
                        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
                control_command->mutable_engage_advice()->set_reason(
                        status.error_message());
            }
        }
        else
        {
            control_command->mutable_engage_advice()->set_advice(
                    apollo::common::EngageAdvice::READY_TO_ENGAGE);
        }
    }
    // check estop
    estop_ = control_conf_.enable_persistent_estop()
                     ? estop_ || local_view_.trajectory().estop().is_estop()
                     : local_view_.trajectory().estop().is_estop();
    if (local_view_.trajectory().estop().is_estop())
    {
        estop_ = true;

        estop_reason_ = "estop from planning : ";
        estop_reason_ += local_view_.trajectory().estop().reason();
    }

    if (local_view_.trajectory().trajectory_point().empty())
    {
        AWARN_EVERY(100) << "planning has no trajectory point. ";
        estop_ = true;

        estop_reason_ =
                "estop for empty planning trajectory, planning headers: " +
                local_view_.trajectory().header().ShortDebugString();

    }

    // valud is 0
    if (FLAGS_enable_gear_drive_negative_speed_protection)
    {
        const double kEpsilon = 0.001;
        auto first_trajectory_point =
                local_view_.trajectory().trajectory_point(0);
        if (local_view_.chassis().gear_location() == Chassis::GEAR_DRIVE &&
            first_trajectory_point.v() < -1 * kEpsilon)
        {
            estop_ = true;
            estop_reason_ = "estop for negative speed when gear_drive";
        }
    }

    if (!estop_)
    {
        if (local_view_.chassis().driving_mode() == Chassis::COMPLETE_MANUAL)
        {
            controller_agent_.Reset();
            AINFO_EVERY(100) << "Reset Controllers in Manual Mode";
        }
        auto debug = control_command->mutable_debug()->mutable_input_debug();
        debug->mutable_localization_header()->CopyFrom(
                local_view_.localization().header());
        debug->mutable_canbus_header()->CopyFrom(
                local_view_.chassis().header());
        debug->mutable_trajectory_header()->CopyFrom(
                local_view_.trajectory().header());

        if (local_view_.trajectory().is_replan())
        {
            latest_replan_trajectory_header_ =
                    local_view_.trajectory().header();
        }

        if (latest_replan_trajectory_header_.has_sequence_num())
        {
            debug->mutable_latest_replan_trajectory_header()->CopyFrom(
                    latest_replan_trajectory_header_);
        }

        // controller agent
        Status status_compute = controller_agent_.ComputeControlCommand(
                &local_view_.localization(), &local_view_.chassis(),
                &local_view_.trajectory(), control_command);
        if (!status_compute.ok())
        {
            AERROR << "Control main function failed"
                   << " with localization: "
                   << local_view_.localization().ShortDebugString()
                   << " with chassis: "
                   << local_view_.chassis().ShortDebugString()
                   << " with trajectory: "
                   << local_view_.trajectory().ShortDebugString()
                   << " with cmd: " << control_command->ShortDebugString()
                   << " status:" << status_compute.error_message();
            estop_ = true;

            estop_reason_ = status_compute.error_message();
            status = status_compute;
        }
    }

    // if planning set estop, then no control process triggered
    if (estop_)
    {
        AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
        // set Estop command
        control_command->set_speed(0);
        control_command->set_throttle(0);
        control_command->set_brake(control_conf_.soft_estop_brake());
        control_command->set_gear_location(Chassis::GEAR_DRIVE);

        AWARN << "estop_reason";
        AWARN << estop_reason_;
    }

    // check signal
    if (local_view_.trajectory().decision().has_vehicle_signal())
    {
        control_command->mutable_signal()->CopyFrom(
                local_view_.trajectory().decision().vehicle_signal());
    }

#if debug_mpc
    if (local_view_.chassis().driving_mode() == Chassis::COMPLETE_MANUAL)
    {
        AINFO << "munual mode";
    }
    else
    {
        AINFO << "auto mode";
    }

    // AINFO << "debug traj";
    // for (size_t i = 0; i < local_view_.trajectory().trajectory_point_size();
    //      i++)
    // {
    //     if (i > 100)
    //     {
    //         break;
    //     }

    //     AINFO << local_view_.trajectory().trajectory_point(i).DebugString();
    // }

    AINFO << "apollo control messege";
    AINFO << control_command->DebugString();
#endif

    return status;
}

int ControlComponent::publish_msg()
{
    apollo::common::util::FillHeader(node_->Name(), &control_command_);
    control_cmd_writer_->Write(control_command_);

#if debug_pure_pursuit
    if (local_view_.chassis().driving_mode() == Chassis::COMPLETE_MANUAL)
    {
        AINFO << "munual mode";
    }
    else
    {
        AINFO << "auto mode";
    }
    AINFO << control_command.DebugString();
#endif

    return 0;
}

Status ControlComponent::CheckInput(LocalView *local_view)
{
    ADEBUG << "Received localization:"
           << local_view->localization().ShortDebugString();
    ADEBUG << "Received chassis:" << local_view->chassis().ShortDebugString();

    if (!local_view->trajectory().estop().is_estop() &&
        local_view->trajectory().trajectory_point().empty())
    {
        AWARN_EVERY(100) << "planning has no trajectory point. ";
        const std::string msg = absl::StrCat(
                "planning has no trajectory point. planning_seq_num:",
                local_view->trajectory().header().sequence_num());

        return Status(ErrorCode::CONTROL_COMPUTE_ERROR, msg);
    }

    for (auto &trajectory_point :
         *local_view->mutable_trajectory()->mutable_trajectory_point())
    {
        if (std::abs(trajectory_point.v()) <
                    control_conf_.minimum_speed_resolution() &&
            std::abs(trajectory_point.a()) <
                    control_conf_.max_acceleration_when_stopped())
        {
            trajectory_point.set_v(0.0);
            trajectory_point.set_a(0.0);
        }
    }

    injector_->vehicle_state()->Update(local_view->localization(),
                                       local_view->chassis());

    return Status::OK();
}

Status ControlComponent::CheckTimestamp(const LocalView &local_view)
{
    if (!control_conf_.enable_input_timestamp_check() ||
        control_conf_.is_control_test_mode())
    {
        ADEBUG << "Skip input timestamp check by gflags.";
        return Status::OK();
    }
    double current_timestamp = Clock::NowInSeconds();
    double localization_diff =
            current_timestamp -
            local_view.localization().header().timestamp_sec();
    if (localization_diff > (control_conf_.max_localization_miss_num() *
                             control_conf_.localization_period()))
    {
        AERROR << "Localization msg lost for " << std::setprecision(6)
               << localization_diff << "s";
        // monitor_logger_buffer_.ERROR("Localization msg lost");
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                      "Localization msg timeout");
    }

    double chassis_diff =
            current_timestamp - local_view.chassis().header().timestamp_sec();
    if (chassis_diff >
        (control_conf_.max_chassis_miss_num() * control_conf_.chassis_period()))
    {
        AERROR << "Chassis msg lost for " << std::setprecision(6)
               << chassis_diff << "s";
        // monitor_logger_buffer_.ERROR("Chassis msg lost");
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
    }

    double trajectory_diff = current_timestamp -
                             local_view.trajectory().header().timestamp_sec();
    if (trajectory_diff > (control_conf_.max_planning_miss_num() *
                           control_conf_.trajectory_period()))
    {
        AERROR << "Trajectory msg lost for " << std::setprecision(6)
               << trajectory_diff << "s";
        // monitor_logger_buffer_.ERROR("Trajectory msg lost");
        return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                      "Trajectory msg timeout");
    }
    return Status::OK();
}
}  // namespace control
}  // namespace apollo