#pragma once

#include "modules/common/status/system_state.h"
#include "opencv_viz.h"

#include <iostream>
#include <memory>


namespace apollo
{
int viz_draw_decision_list(planning::DecisionResult *decisions,
                             viz2d_image *viz2d, const Pose2D*veh_pose);

int viz_draw_main_decision_result(planning::MainDecision *decision,
                                  viz2d_image *viz2d,
                                  const Pose2D*veh_pose);

int viz_draw_main_stop_decision(planning::MainStop *decision,
                             viz2d_image *viz2d,
                             const Pose2D*veh_pose);

int viz_draw_stop_decision(const planning::ObjectStop &decision,
                           viz2d_image *viz2d, const Pose2D*veh_pose,
                           int perception_id);

int viz_draw_common_decision(const planning::ObjectDecisionType &decision,
                             viz2d_image *viz2d, const Pose2D*veh_pose,
                             int perception_id);

int viz_draw_common_decision_list(
        const planning::ObjectDecisions &decision_list, viz2d_image *viz2d,
        const Pose2D*veh_pose);

int viz_draw_follow_decision(const planning::ObjectFollow &decision,
                             viz2d_image *viz2d, const Pose2D*veh_pose,
                             int perception_id);

int viz_draw_yield_decision(const planning::ObjectYield &decision,
                            viz2d_image *viz2d, const Pose2D*veh_pose,
                            int perception_id);

int viz_draw_overtake_decision(const planning::ObjectOvertake &decision,
                               viz2d_image *viz2d, const Pose2D*veh_pose,
                               int perception_id);

int viz_draw_dynamic_scenerio_type(viz2d_image *viz2d,
                                   const Pose2D*veh_pose,
                                   const planning::SpeedDecision &decision);
}