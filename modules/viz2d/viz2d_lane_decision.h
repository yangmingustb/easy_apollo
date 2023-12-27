#pragma once 

#include "modules/common/status/system_state.h"
#include "opencv_viz.h"

#include <iostream>
#include <memory>


namespace apollo
{
int viz_draw_lane_decision(planning::DecisionResult *decisions,
                           viz2d_image *uviz, const Pose2D*veh_pose,
                           const planning::ADCTrajectory &traj);

int viz_draw_lane_borrow_decision(const planning::LaneBorrowStatus &decision,
                                  viz2d_image *uviz,
                                  const Pose2D*veh_pose,
                                  const planning::path_boundary_type path_type);
}