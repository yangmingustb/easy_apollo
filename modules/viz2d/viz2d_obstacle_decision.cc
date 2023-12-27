
#include "viz2d_obstacle_decision.h"
#include "modules/viz2d/viz_window.h"
#include "viz2d_geometry.h"

namespace apollo
{
int viz_draw_decision_list(planning::DecisionResult *decisions,
                           viz2d_image *uviz, const Pose2D*veh_pose)
{
    if (decisions == nullptr)
    {
        return 0;
    }

    if (decisions->has_main_decision())
    {
        viz_draw_main_decision_result(decisions->mutable_main_decision(), uviz,
                                      veh_pose);
    }

    if (decisions->has_object_decision())
    {
        viz_draw_common_decision_list(decisions->object_decision(), uviz,
                                      veh_pose);
    }

    return 0;
}

int viz_draw_common_decision_list(
        const planning::ObjectDecisions &decision_list, viz2d_image *uviz,
        const Pose2D*veh_pose)
{
    if (veh_pose == nullptr || uviz == nullptr)
    {
        return 0;
    }

    for (size_t i = 0; i < decision_list.decision_size(); i++)
    {
        const planning::ObjectDecision &decision = decision_list.decision(i);

        int perception_id = decision.perception_id();
        for (size_t j = 0; j < decision.object_decision_size(); j++)
        {
            const planning::ObjectDecisionType &type =
                    decision.object_decision(j);

            viz_draw_common_decision(type, uviz, veh_pose, perception_id);
        }
    }

    return 0;
}

int viz_draw_common_decision(const planning::ObjectDecisionType &decision,
                             viz2d_image *uviz, const Pose2D*veh_pose,
                             int perception_id)
{
    if (decision.has_stop())
    {
        viz_draw_stop_decision(decision.stop(), uviz, veh_pose, perception_id);
    }
    else if (decision.has_ignore())
    {
    }
    else if (decision.has_follow())
    {
        viz_draw_follow_decision(decision.follow(), uviz, veh_pose,
                                 perception_id);
    }
    else if (decision.has_yield())
    {
        viz_draw_yield_decision(decision.yield(), uviz, veh_pose,
                                perception_id);
    }
    else if (decision.has_overtake())
    {
        viz_draw_overtake_decision(decision.overtake(), uviz, veh_pose,
                                   perception_id);
    }
    else if (decision.has_nudge())
    {
    }
    else if (decision.has_side_pass())
    {
    }

    return 0;
}

int viz_draw_main_decision_result(planning::MainDecision *decision,
                                  viz2d_image *uviz,
                                  const Pose2D*veh_pose)
{
    if (decision->has_stop())
    {
        // viz_draw_main_stop_decision(decision->mutable_stop(), uviz, veh_pose);
    }
    else if (decision->has_change_lane())
    {
    }
    else if (decision->has_cruise())
    {
    }
    else if (decision->has_estop())
    {
    }
    else if (decision->has_mission_complete())
    {
    }
    else if (decision->has_not_ready())
    {
    }
    else if (decision->has_parking())
    {
    }
    else
    {
    }

    return 0;
}

int viz_draw_main_stop_decision(planning::MainStop *decision,
                                viz2d_image *uviz, const Pose2D*veh_pose)
{
    const common::PointENU &stop_point = decision->stop_point();

    Pose2D stop_pose;
    stop_pose.pos.x = stop_point.x();
    stop_pose.pos.y = stop_point.y();
    stop_pose.theta = decision->stop_heading();

    viz2d_color wall_color;

    switch (decision->reason_code())
    {
        case planning::StopReasonCode::STOP_REASON_SIGNAL:
            wall_color = viz2d_colors_red;
            break;

        default:
            wall_color = viz2d_colors_red;
            break;
    }

    viz_draw_virtual_wall(&stop_pose, uviz, veh_pose, 0, wall_color);

    // draw stop reason
    if (decision->has_reason())
    {
        Position2D text_pos_local;

        cvt_pos_global_to_local(&text_pos_local, &stop_pose.pos, veh_pose);

        viz_draw_local_text(uviz, &text_pos_local, decision->reason(),
                            viz2d_colors_cyan);
    }

    return 0;
}

int viz_draw_stop_decision(const planning::ObjectStop &decision,
                           viz2d_image *uviz, const Pose2D*veh_pose,
                           int perception_id)
{
    const common::PointENU &stop_point = decision.stop_point();

    Pose2D stop_pose;
    stop_pose.pos.x = stop_point.x();
    stop_pose.pos.y = stop_point.y();
    stop_pose.theta = decision.stop_heading();

    viz2d_color wall_color;

    switch (decision.reason_code())
    {
        case planning::StopReasonCode::STOP_REASON_SIGNAL:
            wall_color = viz2d_colors_red;
            break;

        default:
            wall_color = viz2d_colors_red;
            break;
    }

    viz_draw_virtual_wall(&stop_pose, uviz, veh_pose, perception_id,
                          wall_color);

    // draw stop reason
    std::string text;
    text = planning::StopReasonCode_Name(decision.reason_code()) +
           std::to_string(perception_id);
    Position2D text_pos_local;

    cvt_pos_global_to_local(&text_pos_local, &stop_pose.pos, veh_pose);

    viz_draw_local_text(uviz, &text_pos_local, text, viz2d_colors_cyan);

    return 0;
}

int viz_draw_follow_decision(const planning::ObjectFollow &decision,
                             viz2d_image *uviz, const Pose2D*veh_pose,
                             int perception_id)
{
    const common::PointENU &stop_point = decision.fence_point();

    Pose2D stop_pose;
    stop_pose.pos.x = stop_point.x();
    stop_pose.pos.y = stop_point.y();
    stop_pose.theta = decision.fence_heading();

    viz2d_color wall_color;

    wall_color = viz2d_colors_green;

    viz_draw_virtual_wall(&stop_pose, uviz, veh_pose, perception_id,
                          wall_color);

    // draw stop reason
    std::string text;
    text = "follow:" + std::to_string(perception_id);
    Position2D text_pos_local;

    cvt_pos_global_to_local(&text_pos_local, &stop_pose.pos, veh_pose);

    viz_draw_local_text(uviz, &text_pos_local, text, viz2d_colors_cyan);

    return 0;
}

int viz_draw_yield_decision(const planning::ObjectYield &decision,
                            viz2d_image *uviz, const Pose2D*veh_pose,
                            int perception_id)
{
    const common::PointENU &stop_point = decision.fence_point();

    Pose2D stop_pose;
    stop_pose.pos.x = stop_point.x();
    stop_pose.pos.y = stop_point.y();
    stop_pose.theta = decision.fence_heading();

    viz2d_color wall_color;

    wall_color = viz2d_colors_yellow;

    viz_draw_virtual_wall(&stop_pose, uviz, veh_pose, perception_id,
                          wall_color);

    // draw stop reason
    std::string text;
    text = "yield:" + std::to_string(perception_id);
    Position2D text_pos_local;

    cvt_pos_global_to_local(&text_pos_local, &stop_pose.pos, veh_pose);

    viz_draw_local_text(uviz, &text_pos_local, text, viz2d_colors_cyan);

    return 0;
}

int viz_draw_overtake_decision(const planning::ObjectOvertake &decision,
                               viz2d_image *uviz, const Pose2D*veh_pose,
                               int perception_id)
{
    const common::PointENU &stop_point = decision.fence_point();

    Pose2D stop_pose;
    stop_pose.pos.x = stop_point.x();
    stop_pose.pos.y = stop_point.y();
    stop_pose.theta = decision.fence_heading();

    viz2d_color wall_color;

    wall_color = viz2d_colors_pink;

    viz_draw_virtual_wall(&stop_pose, uviz, veh_pose, perception_id,
                          wall_color);

    // draw stop reason
    std::string text;
    text = "overtake:" + std::to_string(perception_id);
    Position2D text_pos_local;

    cvt_pos_global_to_local(&text_pos_local, &stop_pose.pos, veh_pose);

    viz_draw_local_text(uviz, &text_pos_local, text, viz2d_colors_cyan);

    return 0;
}

int viz_draw_dynamic_scenerio_type(viz2d_image *uviz,
                                   const Pose2D*veh_pose,
                                   const planning::SpeedDecision &decision)
{
    CvPoint text_center;

    text_center.x = 100;
    text_center.y = 220;

    int text_y_step = 20;

    std::string text;

    // path type
    text_center.y += text_y_step;

    if (decision.has_type())
    {
        text = planning::interaction_secenerio_type_Name(decision.type());
    }
    else
    {
        text = "no dynamic scenario";
    }

    viz_draw_text_relative_to_cv_coordinate(uviz, text, viz2d_colors_white,
                                            text_center);

    // acc
    text_center.y += text_y_step;
    if (decision.has_acc())
    {
        text = "acc: " + std::to_string(decision.acc());
    }
    else
    {
        text = "acc: none ";
    }
    viz_draw_text_relative_to_cv_coordinate(uviz, text, viz2d_colors_white,
                                            text_center);

    // thd
    text_center.y += text_y_step;
    if (decision.has_time_headway())
    {
        text = "thd: " + std::to_string(decision.time_headway());
    }
    else
    {
        text = "thd: none";
    }
    viz_draw_text_relative_to_cv_coordinate(uviz, text, viz2d_colors_white,
                                            text_center);

    // ttc
    text_center.y += text_y_step;
    if (decision.has_ttc())
    {
        text = "ttc: " + std::to_string(decision.ttc());
    }
    else
    {
        text = "ttc: none";
    }
    viz_draw_text_relative_to_cv_coordinate(uviz, text, viz2d_colors_white,
                                            text_center);


    return 0;
}

}  // namespace apollo