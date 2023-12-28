#pragma once


#include "modules/common/status/system_state.h"
#include "opencv_viz.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/routing/routing_component.h"

#include <iostream>
#include <memory>
#include "cyber/common/file.h"
#include "modules/common/util/debug_mode.h"




namespace apollo
{

int viz_draw_traffic_lights(perception::TrafficLightDetection *lights,
                            const Pose2D*veh_pose,
                            viz2d_image *viz2d);

int viz_draw_traffic_light(hdmap::SignalInfoConstPtr signal_info,
                           const Pose2D*veh_pose, viz2d_image *viz2d,
                           viz2d_color color_index, int radius);

int update_obs_list_pose_by_heading(perception::PerceptionObstacles *obs_list,
                                    const Pose2D&base_pose);

int update_obs_list_pose_by_key(perception::PerceptionObstacles *obs_list,
                                bool left, bool right, bool up, bool low);

/**
 * @brief
 * @note
 * @param  &obs_list:
 * @param  &pose:  global pose for obs
 * @retval
 */
int generate_obs_by_cv(
        std::shared_ptr<perception::PerceptionObstacles> &obs_list,
        const Pose2D&pose);

int draw_obs_list(std::shared_ptr<perception::PerceptionObstacles> &obs_list,
                  const Pose2D &base_pose, viz2d_image *window);

int draw_obs_list(perception::PerceptionObstacles *obs_list,
                  const Pose2D&base_pose, viz2d_image *window);

int update_obs_list_pose_by_route(
        perception::PerceptionObstacles *obs_list, const Pose2D &base_pose,
        const std::vector<apollo::hdmap::MapPathPoint> &routing_points_);

int generate_obs_by_cv(perception::PerceptionObstacles *obs_list,
                       const Pose2D &pose, int virtual_obs_speed_type,
                       double virtual_obs_v);

bool remove_obs_by_cv(
        std::shared_ptr<perception::PerceptionObstacles> &obs_list,
        const Position2D &check_pos);

bool remove_obs_by_cv(perception::PerceptionObstacles *obs_list,
                      const Position2D &check_pos);
}