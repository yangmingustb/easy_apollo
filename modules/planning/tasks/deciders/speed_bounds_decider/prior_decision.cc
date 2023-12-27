#include "prior_decision.h"


namespace apollo
{

namespace planning
{

// 为了加速计算，一些显然可以忽略的障碍，可以ignore
bool is_ignore_for_ego_back_object_prior_decision(
        const SLBoundary& adc_sl_bound, const SLBoundary& obs_boundary,
        Obstacle* obstacle, const localization::Pose& veh_pose,
        ReferenceLineInfo* const reference_line_info)
{
    if (obstacle->speed() < 0.1)
    {
        return true;
    }

    double theta = obstacle->Perception().theta();

    double ego_theta = veh_pose.heading();

    double delta_theta = std::fabs(theta - ego_theta);

    if (delta_theta > M_PI_2)
    {
        return true;
    }

    bool is_ego_on_ramp = false;
    hdmap::LaneInfoConstPtr lane =
            reference_line_info->LocateLaneInfo(adc_sl_bound.start_s());

    if (lane != nullptr)
    {
        if (lane->lane().type() == hdmap::Lane::RAMP)
        {
            is_ego_on_ramp = true;
        }
    }

    if (delta_theta > 75.0 * M_PI / 180.0)
    {
        if (obstacle->HasTrajectory())
        {
            int obs_size = obstacle->Trajectory().trajectory_point_size();
            if (obs_size > 0)
            {
                const auto& trajectory_point =
                        obstacle->Trajectory().trajectory_point(obs_size - 1);

                const common::math::Box2d obs_box =
                        obstacle->GetBoundingBox(trajectory_point);

                SLBoundary obs_end_bound;

                reference_line_info->reference_line().GetSLBoundary(
                        obs_box, &obs_end_bound);

                if (obs_end_bound.end_s() + 0.5 < adc_sl_bound.start_s())
                {
                    return true;
                }
            }
        }
    }

    if (obs_boundary.end_s() + 120.0 < adc_sl_bound.start_s())
    {
        return true;
    }

    if (obs_boundary.start_l() - 4.0 > adc_sl_bound.end_l())
    {
        return true;
    }

    if (obs_boundary.end_l() + 4.0 < adc_sl_bound.start_l())
    {
        return true;
    }

    if (obs_boundary.start_l() < adc_sl_bound.end_l() &&
        obs_boundary.end_l() > adc_sl_bound.start_l())
    {
        double obs_l = obs_boundary.end_l() - obs_boundary.start_l();

        if (obs_l < 0.1)
        {
            return true;
        }

        // 计算没重叠的部分
        double no_overlap_l = 0.0;
        if (obs_boundary.end_l() > adc_sl_bound.end_l())
        {
            no_overlap_l = (obs_boundary.end_l() - adc_sl_bound.end_l());
        }
        else if (obs_boundary.start_l() < adc_sl_bound.start_l())
        {
            no_overlap_l = (adc_sl_bound.start_l() - obs_boundary.start_l());
        }

        double overlap_ratio = obs_l - no_overlap_l / obs_l;

        if (overlap_ratio > 0.4)
        {
            return true;
        }
    }

    // obs 在back
    if (!is_ego_on_ramp && !reference_line_info->is_path_lane_borrow() &&
        obs_boundary.end_s() < adc_sl_bound.start_s())
    {
        return true;
    }

    return false;
}

bool is_ignore_for_ego_forward_object_prior_decision(
        const SLBoundary& adc_sl_bound, const SLBoundary& obs_boundary,
        Obstacle* obstacle, const localization::Pose& veh_pose,
        ReferenceLineInfo* const reference_line_info)
{


    double theta = obstacle->Perception().theta();

    double ego_theta = veh_pose.heading();

    double delta_theta = std::fabs(theta - ego_theta);

    if (delta_theta < 10.0 * M_PI / 180.0)
    {
        if (obs_boundary.start_s() - 150.0 > adc_sl_bound.end_s())
        {
            return true;
        }
    }

    return false;
}

int make_prior_object_decision(ReferenceLineInfo* const reference_line_info,
                               PathDecision* const path_decision)
{
    if (reference_line_info == nullptr || path_decision == nullptr)
    {
        AERROR << "input is null";
        return 0;
    }

    const ReferenceLine& reference_line = reference_line_info->reference_line();

    const SLBoundary& adc_sl_bound = reference_line_info->AdcSlBoundary();

    const localization::Pose& veh_pose =
            reference_line_info->vehicle_state().pose();

    adc_direction obs_position;

    double ego_back_error = 3.0;

    bool ignore;

    for (const auto* obstacle : path_decision->obstacles().Items())
    {
        auto* mutable_obstacle = path_decision->Find(obstacle->Id());
        const auto& boundary = mutable_obstacle->PerceptionSLBoundary();

        if (boundary.end_s() + ego_back_error < adc_sl_bound.start_s())
        {
            obs_position = adc_direction::BACK;
        }
        else
        {
            obs_position = adc_direction::FRONT;
        }

        ignore = false;

        switch (obs_position)
        {
            case adc_direction::BACK:

                ignore = is_ignore_for_ego_back_object_prior_decision(
                        adc_sl_bound, boundary, mutable_obstacle, veh_pose,
                        reference_line_info);
                break;
            case adc_direction::FRONT:
                ignore = is_ignore_for_ego_forward_object_prior_decision(
                        adc_sl_bound, boundary, mutable_obstacle, veh_pose,
                        reference_line_info);
                break;
            case adc_direction::LEFT:
                break;
            case adc_direction::RIGHT:
                break;

            default:
                break;
        }

        if (ignore)
        {
            ObjectDecisionType ignore_decision;
            ignore_decision.mutable_ignore();
            if (!mutable_obstacle->HasLongitudinalDecision())
            {
                mutable_obstacle->AddLongitudinalDecision("prior_processor",
                                                          ignore_decision);
            }
            if (!mutable_obstacle->HasLateralDecision())
            {
                mutable_obstacle->AddLateralDecision("prior_processor",
                                                     ignore_decision);
            }
        }
    }

    return 0;
}

}  // namespace planning
}  // namespace apollo