/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/tasks/deciders/speed_decider/speed_decider.h"

#include <algorithm>
#include <memory>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/decision.pb.h"
#include "modules/planning/tasks/utils/st_gap_estimator.h"
#include "modules/planning/tasks/deciders/speed_decider/follow_decider.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/tasks/deciders/speed_decider/yield_decider.h"
#include "modules/planning/tasks/deciders/speed_decider/stop_decider.h"
#include "modules/planning/tasks/deciders/speed_decider/overtake_decider.h"

namespace apollo
{
namespace planning
{
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::perception::PerceptionObstacle;
#define debug_speed_decider (0)
#define debug_dynamics_profile (0)

SpeedDecider::SpeedDecider(
        const TaskConfig& config,
        const std::shared_ptr<DependencyInjector>& injector) :
    Task(config, injector)
{
}

common::Status SpeedDecider::Execute(Frame* frame,
                                     ReferenceLineInfo* reference_line_info)
{
    // init
    Task::Execute(frame, reference_line_info);
    init_point_ = frame_->PlanningStartPoint();
    adc_sl_boundary_ = reference_line_info_->AdcSlBoundary();
    reference_line_ = &reference_line_info_->reference_line();

    adc_sl_point_ = reference_line_info_->get_start_point_sl();

    path_length_ = reference_line_info_->get_lateral_path_length();

    adc_heading_ = reference_line_info_->vehicle_state_heading();

    vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();

    generate_speed_profile(reference_line_info->speed_data(),
                           reference_line_info->path_decision(),
                           reference_line_info);

    // 根据DP结果，更新决策
    if (!MakeObjectDecision(reference_line_info->speed_data(),
                            reference_line_info->path_decision())
                 .ok())
    {
        const std::string msg = "Get object decision by speed profile failed.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    record_constraints(reference_line_info);

    // 识别是否在car follow scenario

    bool has_follow_decision = false;
    bool is_only_following_decision = true;
    double following_min_s = 200.0;
    std::string following_obs_id;

    double stopping_min_s = 200.0;
    std::string stop_obs_id;

    for (const auto* obstacle :
         reference_line_info->path_decision()->obstacles().Items())
    {
#if debug_speed_decider
        AINFO << obstacle->DebugString();

#endif

        if (obstacle->IsVirtual())
        {
            continue;
        }

        if (obstacle->LongitudinalDecision().has_stop())
        {
            is_only_following_decision = false;

            if (obstacle->path_st_boundary().min_s() < stopping_min_s)
            {
                stopping_min_s = obstacle->path_st_boundary().min_s();
                stop_obs_id = obstacle->Id();
            }
        }

        if (obstacle->LongitudinalDecision().has_overtake() ||
            obstacle->LongitudinalDecision().has_yield())
        {
            is_only_following_decision = false;
        }

        if (obstacle->LongitudinalDecision().has_follow())
        {
            has_follow_decision = true;

            if (obstacle->path_st_boundary().min_s() < following_min_s)
            {
                following_min_s = obstacle->path_st_boundary().min_s();
                following_obs_id = obstacle->Id();
            }
        }
    }

#if debug_speed_decider
    AINFO << "is car follow scenario " << has_follow_decision;

#endif

    SpeedDecision speed_decision;
    speed_decision.Clear();

    if (has_follow_decision && is_only_following_decision)
    {
        speed_decision.set_type(INTERACTION_SECENERIO_FOLLOW);

        reference_line_info->set_close_following_obstacle(following_obs_id);

        // update time headway

        double time_headway;

        double adc_speed = reference_line_info->get_veh_linear_velocity();
        if (adc_speed < 0.1)
        {
            time_headway = 1000.0;
        }
        else
        {
            time_headway = following_min_s / adc_speed;
        }

        speed_decision.set_time_headway(time_headway);

        Obstacle* close_following_obs =
                reference_line_info->get_mutable_close_following_obstacle();

        if (close_following_obs != nullptr)
        {

           close_following_obs->set_time_of_headway(time_headway);

           double ttc = 1000.0;

           if (close_following_obs->speed() < adc_speed && adc_speed > 0.1)
           {
               ttc = following_min_s /
                     (adc_speed - close_following_obs->speed());
           }

           close_following_obs->set_time_to_collision(ttc);
           speed_decision.set_ttc(ttc);
        }

        reference_line_info->set_speed_decision(speed_decision);

        AINFO << "print_adc_time_headway:"
              << "(" << time_headway << ","
              << ")";
    }
    else
    {
        reference_line_info->clear_following_obstacle();
    }

    return Status::OK();
}

STLocation SpeedDecider::GetSTLocation(
        const PathDecision* const path_decision, const SpeedData& speed_profile,
        const STBoundary& st_boundary) const
{
    if (st_boundary.IsEmpty())
    {
        return STLocation::BELOW;
    }

    // 判断速度曲线的位置，在obs上方、下方
    STLocation st_location = STLocation::BELOW;
    bool st_position_set = false;
    const double start_t = st_boundary.min_t();
    const double end_t = st_boundary.max_t();

    for (size_t i = 0; i + 1 < speed_profile.size(); ++i)
    {
        const STPoint curr_st(speed_profile[i].s(), speed_profile[i].t());
        const STPoint next_st(speed_profile[i + 1].s(),
                              speed_profile[i + 1].t());

        if (curr_st.t() < start_t && next_st.t() < start_t)
        {
            continue;
        }
        if (curr_st.t() > end_t)
        {
            break;
        }

        if (!FLAGS_use_st_drivable_boundary)
        {
            common::math::LineSegment2d speed_line(curr_st, next_st);
            if (st_boundary.HasOverlap(speed_line))
            {
                ADEBUG << "speed profile cross st_boundaries.";
                st_location = STLocation::CROSS;

                if (!FLAGS_use_st_drivable_boundary)
                {
                    if (st_boundary.boundary_type() ==
                        STBoundary::BoundaryType::KEEP_CLEAR)
                    {
                        if (!CheckKeepClearCrossable(
                                    path_decision, speed_profile, st_boundary))
                        {
                            st_location = STLocation::BELOW;
                        }
                    }
                }
                break;
            }
        }

        // note: st_position can be calculated by checking two st points once
        //       but we need iterate all st points to make sure there is no
        //       CROSS
        if (!st_position_set)
        {
            if (start_t < next_st.t() && curr_st.t() < end_t)
            {
                STPoint bd_point_front = st_boundary.upper_points().front();
                double side = common::math::CrossProd(bd_point_front, curr_st,
                                                      next_st);
                st_location =
                        side < 0.0 ? STLocation::ABOVE : STLocation::BELOW;
                st_position_set = true;
            }
        }
    }
    return st_location;
}

bool SpeedDecider::CheckKeepClearCrossable(
        const PathDecision* const path_decision, const SpeedData& speed_profile,
        const STBoundary& keep_clear_st_boundary) const
{
    bool keep_clear_crossable = true;

    const auto& last_speed_point = speed_profile.back();
    double last_speed_point_v = 0.0;
    if (last_speed_point.has_v())
    {
        last_speed_point_v = last_speed_point.v();
    }
    else
    {
        const size_t len = speed_profile.size();
        if (len > 1)
        {
            const auto& last_2nd_speed_point = speed_profile[len - 2];
            last_speed_point_v =
                    (last_speed_point.s() - last_2nd_speed_point.s()) /
                    (last_speed_point.t() - last_2nd_speed_point.t());
        }
    }

    static constexpr double kKeepClearSlowSpeed = 2.5;  // m/s
    ADEBUG << "last_speed_point_s[" << last_speed_point.s()
           << "] st_boundary.max_s[" << keep_clear_st_boundary.max_s()
           << "] last_speed_point_v[" << last_speed_point_v << "]";

    if (last_speed_point.s() <= keep_clear_st_boundary.max_s() &&
        last_speed_point_v < kKeepClearSlowSpeed)
    {
        keep_clear_crossable = false;
    }
    return keep_clear_crossable;
}

bool SpeedDecider::CheckKeepClearBlocked(
        const PathDecision* const path_decision,
        const Obstacle& keep_clear_obstacle) const
{
    bool keep_clear_blocked = false;

    // check if overlap with other stop wall
    for (const auto* obstacle : path_decision->obstacles().Items())
    {
        if (obstacle->Id() == keep_clear_obstacle.Id())
        {
            continue;
        }
        const double obstacle_start_s =
                obstacle->PerceptionSLBoundary().start_s();
        const double adc_length =
                VehicleConfigHelper::GetConfig().vehicle_param().length();
        const double distance =
                obstacle_start_s -
                keep_clear_obstacle.PerceptionSLBoundary().end_s();

        if (!obstacle->is_keep_clear() && distance > 0 &&
            distance < (adc_length / 2))
        {
            keep_clear_blocked = true;
            break;
        }
    }
    return keep_clear_blocked;
}

bool SpeedDecider::is_adc_need_stop(const Obstacle& obstacle) const
{
    if (obstacle.is_keep_clear())
    {
        return false;
    }

    if (obstacle.path_st_boundary().min_t() > 0.0)
    {
        return false;
    }

    const double obs_speed = obstacle.speed();
    const double ego_speed = init_point_.v();
    if (obs_speed > ego_speed)
    {
        return false;
    }

    if (obs_speed < 1.0)
    {
        return true;
    }
    else
    {
        return false;
    }

    // 安全距离，2* acc * delta_x = v^2 - v_0 ^2；
    // 当自车速度减到障碍物速度，计算刹车距离
    // 
    // FLAGS_min_stop_distance_obstacle， 6
    const double safe_distance = obstacle.path_st_boundary().min_s() -
                                 FLAGS_min_stop_distance_obstacle;

    static constexpr double lane_follow_max_decel = 3.0;
    static constexpr double lane_change_max_decel = 3.0;
    auto* planning_status = injector_->planning_context()
                                    ->mutable_planning_status()
                                    ->mutable_change_lane();

    double delta_v = ego_speed - obs_speed;

    double distance_numerator = std::pow(delta_v, 2) * 0.5;
    double distance_denominator = lane_follow_max_decel;

    // 计算刹车距离
    if (planning_status->has_status() &&
        planning_status->status() == ChangeLaneStatus::IN_CHANGE_LANE)
    {
        distance_denominator = lane_change_max_decel;
    }

    double brake_dist = distance_numerator / distance_denominator;

    return safe_distance < brake_dist;
}

int SpeedDecider::generate_speed_profile(
        const SpeedData& speed_profile, PathDecision* const path_decision,
        ReferenceLineInfo* reference_line_info)
{
    // dp搜索失败
    if (speed_profile.size() < 2)
    {
        const std::string msg = "dp_st_graph failed to get speed profile.";
        AERROR << msg;

        return 0;
    }

    if (path_decision->obstacles().Items().size() < 1)
    {
        AINFO << "No obstacle, so no need do decision";
        return 0;
    }

    SpeedProfileGenerator speed_generator;

    speed_generator.init_speed_bound(init_point_.v());

    hard_brake_speed_data_ =
            reference_line_info->mutable_emergency_brake_speed_data();

    const auto qp_start = std::chrono::system_clock::now();

    // 低速情形下，不要使用qp来生成速度曲线
    if (init_point_.v() < 0.2)
    {
        if (init_point_.v() <= 0.0)
        {
            speed_generator.generate_zero_speed_profile(hard_brake_speed_data_);
        }
        else
        {
            *hard_brake_speed_data_ =
                    speed_generator.generate_constant_jerk_brake_profile(
                            init_point_.v(), init_point_.a(),
                            FLAGS_longitudinal_jerk_lower_bound);
        }
    }
    else
    {
        *hard_brake_speed_data_ =
                speed_generator.generate_constant_jerk_brake_profile_by_qp(
                        init_point_.v(), init_point_.a(),
                        FLAGS_longitudinal_jerk_lower_bound, path_length_,
                        speed_mode::hard_brake);
    }

#if debug_dynamics_profile
    AINFO << "hard brake curve";
    hard_brake_speed_data_->log_speed_data();

#endif

    // generate speed up profile
    speed_up_data_ = speed_generator.speed_sampling_constant_acc(
            init_point_.v(), vehicle_param_.max_acceleration(),
            FLAGS_planning_upper_speed_limit, 7.0);

    const auto qp_end = std::chrono::system_clock::now();
    std::chrono::duration<double> qp_diff = qp_end - qp_start;

    AINFO << "speed qp optimization takes " << qp_diff.count() * 1000.0
          << " ms";

    speed_generator.set_lower_acc(-2.1);
    soft_brake_speed_data_ =
            speed_generator.generate_constant_jerk_brake_profile(
                    init_point_.v(), init_point_.a(),
                    FLAGS_longitudinal_jerk_lower_bound);

    const auto qp_end2 = std::chrono::system_clock::now();
    qp_diff = qp_end2 - qp_end;
    AINFO << "soft brake speed qp optimization takes "
          << qp_diff.count() * 1000.0 << " ms";

    return  0;


}

Status SpeedDecider::MakeObjectDecision(const SpeedData& speed_profile,
                                        PathDecision* const path_decision) const
{
    // dp搜索失败
    if (speed_profile.size() < 2)
    {
        const std::string msg = "dp_st_graph failed to get speed profile.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    if (path_decision->obstacles().Items().size() < 1)
    {
        AINFO << "No obstacle, so no need do decision";
        return Status::OK();
    }

    bool is_reverse_driving_obs = false;

    Obstacle* mutable_obstacle;

    for (const auto* obstacle : path_decision->obstacles().Items())
    {
        mutable_obstacle = path_decision->Find(obstacle->Id());
        const auto& boundary = mutable_obstacle->path_st_boundary();

        if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
            boundary.max_t() < 0.0 ||
            boundary.min_t() >= speed_profile.back().t())
        {
            AppendIgnoreDecision(mutable_obstacle);
            continue;
        }

        // TODO:yangming
        // 如果有纵向决策，增加一个ignore决策
        if (obstacle->HasLongitudinalDecision())
        {
            AppendIgnoreDecision(mutable_obstacle);
            continue;
        }

        // for Virtual obstacle, skip if center point NOT "on lane"
        if (obstacle->IsVirtual())
        {
            const auto& obstacle_box = obstacle->PerceptionBoundingBox();
            if (!reference_line_->IsOnLane(obstacle_box.center()))
            {
                continue;
            }
        }

        // always STOP for pedestrian
        // 行人在移动，adc need stop, return true
        // 行人长时间静止，adc can move forward
        if (CheckStopForPedestrian(*mutable_obstacle))
        {
            ObjectDecisionType stop_decision;
            if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                   -FLAGS_min_stop_distance_obstacle))
            {
                mutable_obstacle->AddLongitudinalDecision(
                        "dp_st_graph/pedestrian", stop_decision);
            }
            continue;
        }

        // 计算st曲线在obstacle的位置
        auto location = GetSTLocation(path_decision, speed_profile, boundary);

        if (!FLAGS_use_st_drivable_boundary)
        {
            if (boundary.boundary_type() ==
                STBoundary::BoundaryType::KEEP_CLEAR)
            {
                if (CheckKeepClearBlocked(path_decision, *obstacle))
                {
                    location = STLocation::BELOW;
                }
            }
        }

#if debug_speed_decider

        AINFO << get_st_location_string(location)
              << ", obs id: " << obstacle->Id();
#endif

        is_reverse_driving_obs =
                is_reverse_driving_obstacle(*obstacle, boundary);

        if (is_reverse_driving_obs)
        {
            mutable_obstacle->set_drive_direction(
                    agent_drive_direction::REVERSE_DRIVING);
        }

        switch (location)
        {
            case STLocation::BELOW:
                if (boundary.boundary_type() ==
                    STBoundary::BoundaryType::KEEP_CLEAR)
                {
                    ObjectDecisionType stop_decision;
                    if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                           0.0))
                    {
                        mutable_obstacle->AddLongitudinalDecision(
                                "dp_st_graph/keep_clear", stop_decision);
                    }
                }
                else if (CheckIsFollow(*obstacle, boundary))
                {
                    // stop for low_speed decelerating
                    if (is_adc_need_stop(*mutable_obstacle))
                    {
                        ObjectDecisionType stop_decision;
                        if (CreateStopDecision(
                                    *mutable_obstacle, &stop_decision,
                                    -FLAGS_min_stop_distance_obstacle))
                        {
                            mutable_obstacle->AddLongitudinalDecision(
                                    "dp_st_graph/too_close", stop_decision);
                        }
                    }
                    else
                    {
                        // high speed or low speed accelerating
                        // FOLLOW decision
                        ObjectDecisionType follow_decision;
                        if (CreateFollowDecision(*mutable_obstacle,
                                                 &follow_decision))
                        {
                            mutable_obstacle->AddLongitudinalDecision(
                                    "dp_st_graph", follow_decision);
                        }
                    }
                }
                else if (is_reverse_driving_obs)
                {
                    AINFO << "reverse obs, "
                          << ", obs id: " << obstacle->Id()
                          << ", st curve is below";

                    // will be retired if bev perception fix reverse obs
                    continue;

                    ObjectDecisionType stop_decision;

                    if (create_stop_decision_for_reverse_obs(*mutable_obstacle,
                                                             &stop_decision))
                    {
                        mutable_obstacle->AddLongitudinalDecision(
                                "dp_st_graph/stop_by_reverse_obs",
                                stop_decision);
                    }
                }
                else
                {
                    const auto& obstacle_boundary =
                            obstacle->PerceptionSLBoundary();

                    // overtake or ignore,
                    if (obstacle_boundary.end_s() < adc_sl_boundary_.start_s())
                    {
                        continue;
                    }

                    // YIELD decision
                    ObjectDecisionType yield_decision;
                    if (CreateYieldDecision(*mutable_obstacle, &yield_decision))
                    {
                        mutable_obstacle->AddLongitudinalDecision(
                                "dp_st_graph", yield_decision);
                    }
                }
                break;
            case STLocation::ABOVE:
                if (boundary.boundary_type() ==
                    STBoundary::BoundaryType::KEEP_CLEAR)
                {
                    ObjectDecisionType ignore;
                    ignore.mutable_ignore();
                    mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                              ignore);
                }
                else
                {
                    // OVERTAKE decision
                    // 这里超车一定能够成功吗：

                    // gap is small, ignore
                    double overtake_gap;
                    estimate_min_lon_gap_for_overtake_decision(
                            &overtake_gap, *mutable_obstacle, &speed_up_data_);

                    if (overtake_gap < 5.0)
                    {
                        AINFO << "can not overtale obs id: " << obstacle->Id();
                        break;
                    }

                    ObjectDecisionType overtake_decision;
                    if (CreateOvertakeDecision(*mutable_obstacle,
                                               &overtake_decision))
                    {
                        mutable_obstacle->AddLongitudinalDecision(
                                "dp_st_graph/overtake", overtake_decision);
                    }
                }
                break;
            case STLocation::CROSS:

                if (is_reverse_driving_obs)
                {
                    AINFO << "reverse obs, "
                          << ", obs id: " << obstacle->Id()
                          << ", st curve is cross";

                    // will be retired if bev perception fix reverse obs
                    continue;

                    ObjectDecisionType stop_decision;

                    if (create_stop_decision_for_reverse_obs(*mutable_obstacle,
                                                             &stop_decision))
                    {
                        mutable_obstacle->AddLongitudinalDecision(
                                "dp_st_graph/stop_by_reverse_obs",
                                stop_decision);
                    }
                }
                else if (!mutable_obstacle->is_keep_clear())
                {
                    ObjectDecisionType stop_decision;
                    if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                           -FLAGS_min_stop_distance_obstacle))
                    {
                        mutable_obstacle->AddLongitudinalDecision(
                                "dp_st_graph/cross", stop_decision);
                    }

                    const std::string msg = absl::StrCat(
                            "Failed to find a solution for crossing obstacle: ",
                            mutable_obstacle->Id());
                    AERROR << msg;
                    
                    // 不需要return fail, 添加一个stop decision即可
                    // return Status(ErrorCode::PLANNING_ERROR, msg);
                }

                break;
            default:
                AERROR << "Unknown position:";
        }

        AppendIgnoreDecision(mutable_obstacle);

    }

    return Status::OK();
}

void SpeedDecider::AppendIgnoreDecision(Obstacle* obstacle) const
{
    ObjectDecisionType ignore_decision;
    ignore_decision.mutable_ignore();
    if (!obstacle->HasLongitudinalDecision())
    {
        obstacle->AddLongitudinalDecision("dp_st_graph", ignore_decision);
    }
    if (!obstacle->HasLateralDecision())
    {
        obstacle->AddLateralDecision("dp_st_graph", ignore_decision);
    }
}

bool SpeedDecider::CreateStopDecision(const Obstacle& obstacle,
                                      ObjectDecisionType* const stop_decision,
                                      double stop_distance) const
{
    const auto& st_boundary = obstacle.path_st_boundary();
    const SLBoundary& obs_sl_boundary = obstacle.PerceptionSLBoundary();

    // TODO(all): this is a bug! Cannot mix reference s and path s!
    // Replace boundary.min_s() with computed reference line s
    // fence is set according to reference line s.

    // double front_edge_to_center;
    // front_edge_to_center = vehicle_param_.front_edge_to_center();

    double ego_front_fence_ref_s = obs_sl_boundary.start_s() + stop_distance;

    // efence不能在ego后方

    ego_front_fence_ref_s = std::max(0.0, ego_front_fence_ref_s);

    if (st_boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR)
    {
        ego_front_fence_ref_s = obstacle.PerceptionSLBoundary().start_s();
    }

    const double main_stop_s =
            reference_line_info_->path_decision()->stop_reference_line_s();
    if (main_stop_s < ego_front_fence_ref_s)
    {
#if debug_speed_decider
        AINFO << "Stop fence is further away, ignore.";

#endif
        return false;
    }

    const auto ego_front_fence_point =
            reference_line_->GetReferencePoint(ego_front_fence_ref_s);

    // set STOP decision
    auto* stop = stop_decision->mutable_stop();
    stop->set_distance_s(stop_distance);
    auto* stop_point = stop->mutable_stop_point();
    stop_point->set_x(ego_front_fence_point.x());
    stop_point->set_y(ego_front_fence_point.y());
    stop_point->set_z(0.0);
    stop->set_stop_heading(ego_front_fence_point.heading());

    if (st_boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR)
    {
        stop->set_reason_code(StopReasonCode::STOP_REASON_CLEAR_ZONE);
    }

#if debug_speed_decider
    PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
    
    AINFO << "STOP: obstacle_id[" << obstacle.Id() << "] obstacle_type["
           << PerceptionObstacle_Type_Name(obstacle_type) << "]";

#endif

    return true;
}

bool SpeedDecider::create_stop_decision_for_reverse_obs(
        const Obstacle& obstacle, ObjectDecisionType* const stop_decision) const
{
    const auto& st_boundary = obstacle.path_st_boundary();
    const SLBoundary& obs_sl_boundary = obstacle.PerceptionSLBoundary();

    double ego_front_path_s;

    estimate_stop_dist_for_reverse_driving_obs(&ego_front_path_s, obstacle,
                                               init_point_.v());

    double ego_front_fence_ref_s =
            adc_sl_boundary_.start_s() + ego_front_path_s;

    // efence不能在ego后方

    ego_front_fence_ref_s = std::max(0.0, ego_front_fence_ref_s);

    const double main_stop_s =
            reference_line_info_->path_decision()->stop_reference_line_s();
    if (main_stop_s < ego_front_fence_ref_s)
    {
#if debug_speed_decider
        AINFO << "Stop fence is further away, ignore.";

#endif
        return false;
    }

    const auto ego_front_fence_point =
            reference_line_->GetReferencePoint(ego_front_fence_ref_s);

    // set STOP decision
    auto* stop = stop_decision->mutable_stop();

    double ego_front_to_obs = obs_sl_boundary.start_s() - ego_front_fence_ref_s;

    stop->set_distance_s(-ego_front_to_obs);

    auto* stop_point = stop->mutable_stop_point();
    stop_point->set_x(ego_front_fence_point.x());
    stop_point->set_y(ego_front_fence_point.y());
    stop_point->set_z(0.0);
    stop->set_stop_heading(ego_front_fence_point.heading());

#if debug_speed_decider
    PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
    
    AINFO << "STOP: obstacle_id[" << obstacle.Id() << "] obstacle_type["
           << PerceptionObstacle_Type_Name(obstacle_type) << "]";

    AINFO << "ego_front_path_s: " << ego_front_path_s;

#endif

    return true;
}

bool SpeedDecider::CreateFollowDecision(
        const Obstacle& obstacle,
        ObjectDecisionType* const follow_decision) const
{
    const double follow_speed = init_point_.v();

    ObjectFollow* follow_info = follow_decision->mutable_follow();

    // 跟车距离是一个线性模型，以后根据舒适的acc计算一个跟车距离,
    // 计算ego车头和障碍物应当保持的距离
    // 注意，这里都计算ego车头和障碍物保持的
    // const double follow_distance_s =
    //         -StGapEstimator::EstimateProperFollowingGap(follow_speed);
    double follow_distance_s;

    estimate_proper_following_gap(
            follow_info, obstacle, follow_speed, init_point_.a(), path_length_,
            hard_brake_speed_data_, &soft_brake_speed_data_, adc_sl_boundary_,
            reference_line_info_->get_max_speed());

    follow_distance_s = follow_info->distance_s();

    const auto& boundary = obstacle.path_st_boundary();

    // 跟车时，车头的s

    const double ego_front_fence_ref_s =
            adc_sl_boundary_.end_s() + boundary.min_s() + follow_distance_s;

    const double main_stop_s =
            reference_line_info_->path_decision()->stop_reference_line_s();
    if (main_stop_s < ego_front_fence_ref_s)
    {
        ADEBUG << "Follow reference_s is further away, ignore.";
        return false;
    }

    auto ref_point = reference_line_->GetReferencePoint(ego_front_fence_ref_s);

    // set FOLLOW decision

    auto* fence_point = follow_info->mutable_fence_point();
    fence_point->set_x(ref_point.x());
    fence_point->set_y(ref_point.y());
    fence_point->set_z(0.0);
    follow_info->set_fence_heading(ref_point.heading());

#if debug_speed_decider
    PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
    AINFO << "FOLLOW: obstacle_id[" << obstacle.Id() << "] obstacle_type["
           << PerceptionObstacle_Type_Name(obstacle_type) << "]";
#endif
    return true;
}

bool SpeedDecider::CreateYieldDecision(
        const Obstacle& obstacle,
        ObjectDecisionType* const yield_decision) const
{
    PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
    double yield_distance;

    ObjectYield* yield_info = yield_decision->mutable_yield();

    estimate_proper_yield_poi(yield_info, obstacle, init_point_.v(),
                              init_point_.a(), path_length_,
                              hard_brake_speed_data_, &soft_brake_speed_data_,
                              adc_sl_boundary_);

    const auto& obstacle_boundary = obstacle.path_st_boundary();

    // 理想的避让距离是6米，如果和前车小于6米，避让距离就是ego和obs距离
    // 计算车头的距离
    const double ego_front_yield_distance_s = yield_info->distance_s();

    const double reference_line_fence_s = adc_sl_boundary_.end_s() +
                                          obstacle_boundary.min_s() +
                                          ego_front_yield_distance_s;

    const double main_stop_s =
            reference_line_info_->path_decision()->stop_reference_line_s();
    if (main_stop_s < reference_line_fence_s)
    {
        ADEBUG << "Yield reference_s is further away, ignore.";
        return false;
    }

    auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

    // set YIELD decision

    yield_info->mutable_fence_point()->set_x(ref_point.x());
    yield_info->mutable_fence_point()->set_y(ref_point.y());
    yield_info->mutable_fence_point()->set_z(0.0);
    yield_info->set_fence_heading(ref_point.heading());

#if debug_speed_decider
    AINFO << "YIELD: obstacle_id[" << obstacle.Id() << "] obstacle_type["
           << PerceptionObstacle_Type_Name(obstacle_type) << "]";

#endif

    return true;
}

bool SpeedDecider::CreateOvertakeDecision(
        const Obstacle& obstacle,
        ObjectDecisionType* const overtake_decision) const
{
    const auto& velocity = obstacle.Perception().velocity();
    const double obstacle_speed =
            common::math::Vec2d::CreateUnitVec2d(
                    init_point_.path_point().theta())
                    .InnerProd(Vec2d(velocity.x(), velocity.y()));

    const double overtake_distance_s =
            StGapEstimator::EstimateProperOvertakingGap(obstacle_speed,
                                                        init_point_.v());

    // 应该使用obs upper s 开始计算， todo
    const auto& boundary = obstacle.path_st_boundary();
    const double reference_line_fence_s =
            adc_sl_boundary_.end_s() + boundary.min_s() + overtake_distance_s;

    const double main_stop_s =
            reference_line_info_->path_decision()->stop_reference_line_s();

    if (main_stop_s < reference_line_fence_s)
    {
        ADEBUG << "Overtake reference_s is further away, ignore.";
        return false;
    }

    auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

    // set OVERTAKE decision
    auto* overtake = overtake_decision->mutable_overtake();
    overtake->set_distance_s(overtake_distance_s);
    overtake->mutable_fence_point()->set_x(ref_point.x());
    overtake->mutable_fence_point()->set_y(ref_point.y());
    overtake->mutable_fence_point()->set_z(0.0);
    overtake->set_fence_heading(ref_point.heading());

#if debug_speed_decider
    PerceptionObstacle::Type obstacle_type = obstacle.Perception().type();
    AINFO << "OVERTAKE: obstacle_id[" << obstacle.Id() << "] obstacle_type["
           << PerceptionObstacle_Type_Name(obstacle_type) << "]";

#endif

    return true;
}

// 2 car 需要同向行驶
bool SpeedDecider::CheckIsFollow(const Obstacle& obstacle,
                                 const STBoundary& boundary) const
{
    const double obstacle_l_distance =
            std::min(std::fabs(obstacle.PerceptionSLBoundary().start_l()),
                     std::fabs(obstacle.PerceptionSLBoundary().end_l()));

    // FLAGS_follow_min_obs_lateral_distance：2.5
    if (obstacle_l_distance > FLAGS_follow_min_obs_lateral_distance)
    {
        return false;
    }

    // move towards adc， reverse obs, 不能follow
    if (boundary.bottom_left_point().s() > boundary.bottom_right_point().s())
    {
        return false;
    }

    static constexpr double kFollowTimeEpsilon = 1e-3;
    static constexpr double kFollowCutOffTime = 0.5;

    // 交互起点时间，超过0.5秒，不follow
    // 如果obs一直在ego前方，那么boundary.min_t() 是0
    // 如果min_t()不是0，说明obs可能是cut in或者 cross， 或者merge车道场景带来的
    // st bounday
    if (boundary.min_t() > kFollowCutOffTime ||
        boundary.max_t() < kFollowTimeEpsilon)
    {
        return false;
    }

    // cross lane but be moving to different direction
    // FLAGS_follow_min_time_sec:同向行驶超过2秒
    // 如果交互时间很小，不要follow
    if (boundary.max_t() - boundary.min_t() < FLAGS_follow_min_time_sec)
    {
        return false;
    }

    return true;
}

bool SpeedDecider::is_reverse_driving_obstacle(const Obstacle& obstacle,
                                               const STBoundary& boundary) const
{

    // check s
    if (boundary.bottom_left_point().s() <= boundary.bottom_right_point().s())
    {
        return false;
    }

    // check reverse driving time
    if (boundary.max_t() - boundary.min_t() < 0.5)
    {
        return false;
    }

    // check speed
    if (obstacle.speed() <= 0.001)
    {
        return false;
    }

    if (obstacle.IsVirtual() || obstacle.IsIgnore())
    {
        return false;
    }

    const prediction::Trajectory& traj = obstacle.Trajectory();
    if (traj.trajectory_point_size() <= 0)
    {
        return false;
    }

    // check start heading
    common::TrajectoryPoint poi_start =
            obstacle.GetPointAtTime(boundary.min_t());

    double heading_diff = adc_heading_ - poi_start.path_point().theta();

    heading_diff = apollo_unify_theta(heading_diff, apollo_PI);

    heading_diff = std::fabs(heading_diff);

    if (heading_diff < 90.0 * M_PI / 180.0)
    {
#if debug_speed_decider
        AINFO << "theta diff is less 90 degree, not reverse obs";

#endif
        return false;
    }

    // check middle heading

    common::TrajectoryPoint poi_middle = obstacle.GetPointAtTime(
            (boundary.min_t() + boundary.max_t()) / 2.0);

    heading_diff = adc_heading_ - poi_middle.path_point().theta();

    heading_diff = apollo_unify_theta(heading_diff, apollo_PI);

    heading_diff = std::fabs(heading_diff);

    if (heading_diff < 120.0 * M_PI / 180.0)
    {
#if debug_speed_decider
        AINFO << "theta diff is less 120 degree, not reverse obs";

#endif
        return false;
    }

    // check end poi heading
    common::TrajectoryPoint poi_end = obstacle.GetPointAtTime(boundary.max_t());

    heading_diff = adc_heading_ - poi_end.path_point().theta();

    heading_diff = apollo_unify_theta(heading_diff, apollo_PI);
    heading_diff = std::fabs(heading_diff);

    if (heading_diff < 145.0 * M_PI / 180.0)
    {
#if debug_speed_decider
        AINFO << "theta diff is less 145 degree, not reverse obs";

#endif
        return false;
    }

    return true;
}

bool SpeedDecider::CheckStopForPedestrian(const Obstacle& obstacle) const
{
    const auto& perception_obstacle = obstacle.Perception();
    if (perception_obstacle.type() != PerceptionObstacle::PEDESTRIAN)
    {
        return false;
    }

    const auto& obstacle_sl_boundary = obstacle.PerceptionSLBoundary();
    if (obstacle_sl_boundary.end_s() < adc_sl_boundary_.start_s())
    {
        return false;
    }

    // read pedestrian stop time from PlanningContext
    auto* mutable_speed_decider_status = injector_->planning_context()
                                                 ->mutable_planning_status()
                                                 ->mutable_speed_decider();

    // 记录静止行人的停止时间
    std::unordered_map<std::string, double> stop_time_map;
    for (const auto& pedestrian_stop_time :
         mutable_speed_decider_status->pedestrian_stop_time())
    {
        stop_time_map[pedestrian_stop_time.obstacle_id()] =
                pedestrian_stop_time.stop_timestamp_sec();
    }

    const std::string& obstacle_id = obstacle.Id();

    // update stop timestamp on static pedestrian for watch timer
    // check on stop timer for static pedestrians
    static constexpr double kSDistanceStartTimer = 10.0;
    static constexpr double kMaxStopSpeed = 0.3;
    static constexpr double kPedestrianStopTimeout = 4.0;

    bool ego_need_to_stop = true;

    if (obstacle.path_st_boundary().min_s() < kSDistanceStartTimer)
    {
        const auto obstacle_speed =
                std::hypot(perception_obstacle.velocity().x(),
                           perception_obstacle.velocity().y());

        // 如果行人移动，删除
        if (obstacle_speed > kMaxStopSpeed)
        {
            stop_time_map.erase(obstacle_id);
        }
        else
        {
            // 新的行人
            if (stop_time_map.count(obstacle_id) == 0)
            {
                // add timestamp
                stop_time_map[obstacle_id] = Clock::NowInSeconds();
                
                ADEBUG << "add timestamp: obstacle_id[" << obstacle_id
                       << "] timestamp[" << Clock::NowInSeconds() << "]";
            }
            // 旧的行人
            else
            {
                // check timeout
                double stop_timer =
                        Clock::NowInSeconds() - stop_time_map[obstacle_id];
                
                ADEBUG << "stop_timer: obstacle_id[" << obstacle_id
                       << "] stop_timer[" << stop_timer << "]";

                if (stop_timer >= kPedestrianStopTimeout)
                {
                    ego_need_to_stop = false;
                }
            }
        }
    }

    // write pedestrian stop time to PlanningContext
    mutable_speed_decider_status->mutable_pedestrian_stop_time()->Clear();
    for (const auto& stop_time : stop_time_map)
    {
        auto pedestrian_stop_time =
                mutable_speed_decider_status->add_pedestrian_stop_time();
        pedestrian_stop_time->set_obstacle_id(stop_time.first);
        pedestrian_stop_time->set_stop_timestamp_sec(stop_time.second);
    }

    // 如果这个行人一直不动，那么车辆不需要增加一个stop decision
    return ego_need_to_stop;
}

int SpeedDecider::record_constraints(ReferenceLineInfo* reference_line_info)
{
    if (soft_brake_speed_data_.size() < 1)
    {
        return 0;
    }

    auto* debug = reference_line_info_->mutable_debug();

    auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();

    ptr_speed_plan->set_name("soft_brake_speed_profile");
    ptr_speed_plan->mutable_speed_point()->CopyFrom(
            {soft_brake_speed_data_.begin(), soft_brake_speed_data_.end()});

    return 0;
}

}  // namespace planning
}  // namespace apollo
