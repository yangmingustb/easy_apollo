
#include "viz2d_lane_decision.h"

namespace apollo
{
int viz_draw_lane_decision(planning::DecisionResult *decisions,
                           viz2d_image *uviz, const Pose2D*veh_pose,
                           const planning::ADCTrajectory &traj)
{
    if (decisions == nullptr)
    {
        return 0;
    }

    if (decisions->has_lane_borrow_decider())
    {
        viz_draw_lane_borrow_decision(decisions->lane_borrow_decider(), uviz,
                                      veh_pose, traj.path_type());
    }


    return 0;
}

int viz_draw_lane_borrow_decision(const planning::LaneBorrowStatus &decision,
                                  viz2d_image *uviz,
                                  const Pose2D*veh_pose,
                                  const planning::path_boundary_type path_type)
{
    CvPoint text_center;

    text_center.x = 100;
    text_center.y = 100;

    int text_y_step = 20;

    std::string text;

    // update state
    text = "lane_borrow_state: ";

    if (decision.is_in_path_lane_borrow_scenario())
    {
        text += " true";
    }
    else
    {
        text += "false";
    }

    viz_draw_text_relative_to_cv_coordinate(uviz, text, viz2d_colors_white,
                                            text_center);


    // path type
    text_center.y += text_y_step;

    text = planning::path_boundary_type_Name(path_type);
    viz_draw_text_relative_to_cv_coordinate(uviz, text, viz2d_colors_white,
                                            text_center);

    // static obs id
    text_center.y += text_y_step;
    if (decision.has_front_static_obstacle_id())
    {
        text = "blocking obs id: " + decision.front_static_obstacle_id() +
               ", counter: " +
               std::to_string(decision.front_static_obstacle_cycle_counter());
    }
    else
    {
        text = "blocking obs id: none ";
    }
    viz_draw_text_relative_to_cv_coordinate(uviz, text, viz2d_colors_white,
                                            text_center);

    // update time
    const planning::LaneBorrowByTurtleVehicle &lane_borrow_turle_car =
            decision.lane_borrow_by_turtle();

    if (lane_borrow_turle_car.has_lane_borrow_counter())
    {
        text = "lane borrow time by turtle car: " +
               std::to_string(lane_borrow_turle_car.lane_borrow_counter() *
                              0.1);
    }
    else
    {
        text = "lane borrow time by turtle car: 0";
    }

    text_center.y += text_y_step;
    viz_draw_text_relative_to_cv_coordinate(uviz, text, viz2d_colors_white,
                                            text_center);

    // update reason

    if (decision.has_lane_borrow_reason())
    {
        text = planning::LaneBorrowReason_Name(decision.lane_borrow_reason());
    }
    else
    {
        text = "none lane borrow";
    }
    text_center.y += text_y_step;
    viz_draw_text_relative_to_cv_coordinate(uviz, text, viz2d_colors_white,
                                            text_center);

    // update lane borrow stage

    if (decision.has_status())
    {
        text = planning::LaneBorrowStateType_Name(decision.status());
    }
    else
    {
        text = "No lane borrow stage";
    }
    text_center.y += text_y_step;
    viz_draw_text_relative_to_cv_coordinate(uviz, text, viz2d_colors_white,
                                            text_center);

    return 0;
}
}