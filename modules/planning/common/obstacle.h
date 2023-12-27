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

#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/indexed_list.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/proto/decision.pb.h"
#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/common/math/polygon_base.h"
#include "modules/planning/common/interaction_data.h"

namespace apollo
{
namespace planning
{

/**
 * @class Obstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 *
 * Lateral decision includes: nudge, ignore.
 * Lateral decision safety priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision safety priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 *
 * obs type有虚拟的：traffic light, keep clear, dead end, terminal
 * point，但是好像没有标记出来.
 *
 * 如何区分：side pass, nudge， side pass没有使用， 2者意义相同
 * 
 * 一个感知对象，可以预测多条轨迹，最后生成多个Obstacle
 */
class Obstacle
{
public:
    Obstacle() = default;
    Obstacle(const std::string& id,
             const perception::PerceptionObstacle& perception_obstacle,
             const prediction::ObstaclePriority::Priority& obstacle_priority =
                     prediction::ObstaclePriority::Priority::
                             ObstaclePriority_Priority_NORMAL,
             const bool is_static = true);
    Obstacle(const std::string& id,
             const perception::PerceptionObstacle& perception_obstacle,
             const prediction::Trajectory& trajectory,
             const prediction::ObstaclePriority::Priority& obstacle_priority =
                     prediction::ObstaclePriority::Priority::
                             ObstaclePriority_Priority_NORMAL,
             const bool is_static = true);

    const std::string& Id() const { return id_; }
    void SetId(const std::string& id) { id_ = id; }

    double speed() const { return speed_; }

    int32_t PerceptionId() const { return perception_id_; }

    bool IsStatic() const { return is_static_; }
    bool IsVirtual() const { return is_virtual_; }

    common::TrajectoryPoint GetPointAtTime(const double time) const;

    common::math::Box2d GetBoundingBox(
            const common::TrajectoryPoint& point) const;

    Polygon2D get_local_polygon() const;

    int get_trajectory_point_polygon(
            Polygon2D* global_polygon, const Polygon2D& local_polygon,
            const common::TrajectoryPoint& point) const;

    const common::math::Box2d& PerceptionBoundingBox() const
    {
        return perception_bounding_box_;
    }
    const common::math::Polygon2d& PerceptionPolygon() const
    {
        return perception_polygon_;
    }
    const prediction::Trajectory& Trajectory() const { return trajectory_; }
    common::TrajectoryPoint* AddTrajectoryPoint()
    {
        return trajectory_.add_trajectory_point();
    }
    bool HasTrajectory() const
    {
        return !(trajectory_.trajectory_point().empty());
    }

    const perception::PerceptionObstacle& Perception() const
    {
        return perception_obstacle_;
    }

    /**
     * @brief This is a helper function that can create obstacles from
     * prediction data.  The original prediction may have multiple trajectories
     * for each obstacle. But this function will create one obstacle for each
     * trajectory.
     * @param predictions The prediction results
     * @return obstacles The output obstacles saved in a list of unique_ptr.
     */
    static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
            const prediction::PredictionObstacles& predictions);

    static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
            const std::string& id, const common::math::Box2d& obstacle_box);

    static bool IsValidPerceptionObstacle(
            const perception::PerceptionObstacle& obstacle);

    static bool IsValidTrajectoryPoint(const common::TrajectoryPoint& point);

    inline bool IsCautionLevelObstacle() const
    {
        return is_caution_level_obstacle_;
    }

    // const Obstacle* obstacle() const;

    /**
     * return the merged lateral decision
     * Lateral decision is one of {Nudge, Ignore}
     **/
    const ObjectDecisionType& LateralDecision() const;

    /**
     * @brief return the merged longitudinal decision
     * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
     **/
    const ObjectDecisionType& LongitudinalDecision() const;

    std::string DebugString() const;

    const SLBoundary& PerceptionSLBoundary() const;

    const STBoundary& reference_line_st_boundary() const;

    const STBoundary& path_st_boundary() const;

    const std::vector<std::string>& decider_tags() const;

    const std::vector<ObjectDecisionType>& decisions() const;

    void AddLongitudinalDecision(const std::string& decider_tag,
                                 const ObjectDecisionType& decision);

    /**
     * @brief
     * @note
     * @param  decider_tag:  决策标记，可以表示是哪一个模块的决策
     * @param  decision:
     * @retval None
     */
    void AddLateralDecision(const std::string& decider_tag,
                            const ObjectDecisionType& decision);

    bool HasLateralDecision() const;

    void set_path_st_boundary(const STBoundary& boundary);

    bool is_path_st_boundary_initialized()
    {
        return path_st_boundary_initialized_;
    }

    void SetStBoundaryType(const STBoundary::BoundaryType type);

    void EraseStBoundary();

    void SetReferenceLineStBoundary(const STBoundary& boundary);

    void SetReferenceLineStBoundaryType(const STBoundary::BoundaryType type);

    void EraseReferenceLineStBoundary();

    bool HasLongitudinalDecision() const;

    bool HasNonIgnoreDecision() const;

    /**
     * @brief Calculate stop distance with the obstacle using the ADC's minimum
     * turning radius
     */
    double MinRadiusStopDistance(
            const common::VehicleParam& vehicle_param) const;

    /**
     * @brief Check if this object can be safely ignored.
     * The object will be ignored if the lateral decision is ignore and the
     * longitudinal decision is ignore
     *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
     */
    bool IsIgnore() const;
    bool IsLongitudinalIgnore() const;
    bool IsLateralIgnore() const;

    void BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
                                      const double adc_start_s);

    void SetPerceptionSlBoundary(const SLBoundary& sl_boundary);

    /**
     * @brief check if an ObjectDecisionType is a longitudinal decision.
     **/
    static bool IsLongitudinalDecision(const ObjectDecisionType& decision);

    /**
     * @brief check if an ObjectDecisionType is a lateral decision.
     **/
    static bool IsLateralDecision(const ObjectDecisionType& decision);

    void set_keep_clear(bool is_keep_clear)
    {
        is_keep_clear_ = is_keep_clear;
    }

    bool is_keep_clear() const { return is_keep_clear_; }

    void set_time_to_collision(const double& ttc)
    {
        time_to_collision_ = ttc;
    }

    const double get_time_to_collision() const { return time_to_collision_; }

    void set_time_of_headway(const double& time) { time_of_headway_ = time; }

    const double get_time_of_headway() const { return time_of_headway_; }

    /*
     * @brief IsLaneBlocking is only meaningful when IsStatic() == true.
     */
    bool IsLaneBlocking() const { return is_lane_blocking_; }
    void CheckLaneBlocking(const ReferenceLine& reference_line);
    bool IsLaneChangeBlocking() const { return is_lane_change_blocking_; }
    void SetLaneChangeBlocking(const bool is_distance_clear);

    void set_drive_direction(const agent_drive_direction& dir)
    {
        drive_dir_ = dir;
        return;
    }

    bool is_reverse_driving()
    {
        if (drive_dir_ == agent_drive_direction::REVERSE_DRIVING)
        {
            return true;
        }

        return false;
    }

private:
    FRIEND_TEST(MergeLongitudinalDecision, AllDecisions);

    static ObjectDecisionType MergeLongitudinalDecision(
            const ObjectDecisionType& lhs, const ObjectDecisionType& rhs);
    
    FRIEND_TEST(MergeLateralDecision, AllDecisions);
    
    static ObjectDecisionType MergeLateralDecision(
            const ObjectDecisionType& lhs, const ObjectDecisionType& rhs);

    bool BuildTrajectoryStBoundary(const ReferenceLine& reference_line,
                                   const double adc_start_s,
                                   STBoundary* const st_boundary);
    
    bool IsValidObstacle(
            const perception::PerceptionObstacle& perception_obstacle);

private:
    // perception id
    std::string id_;
    int32_t perception_id_ = 0;
    bool is_static_ = false;

    // 虚拟障碍物的种类：
    // if PerceptionObstacle id小于0， 就是虚拟
    // keep clear
    // ReferenceLineEnd
    // crosswalk
    // stop sign
    // traffic light
    // ..., 太多了
    bool is_virtual_ = false;
    double speed_ = 0.0;

    bool path_st_boundary_initialized_ = false;

    // prediction traj
    prediction::Trajectory trajectory_;
    perception::PerceptionObstacle perception_obstacle_;

    // east north ?
    // ego local coordinate?
    // east north
    common::math::Box2d perception_bounding_box_;
    common::math::Polygon2d perception_polygon_;

    // 存储所有的决策
    std::vector<ObjectDecisionType> decisions_;
    // 决策标志，哪个decider输出的决策
    std::vector<std::string> decider_tags_;

    // perpception box在ref line形成的sl bound
    SLBoundary sl_boundary_;

    // obs在参考线上投影，使用veh width作为ref line的宽度，生成的st boundary
    // 注意这里的st bound和path st bound不同，s值也不同
    STBoundary reference_line_st_boundary_;

    // path 和 obs形成的st bound
    STBoundary path_st_boundary_;

    ObjectDecisionType lateral_decision_;
    ObjectDecisionType longitudinal_decision_;

    // 目前，if is keep clear obs, then false；
    // 至于其他的虚拟obs, 如 red light, stop sign, termination，是true
    // real static car, true
    // if is dynamic, true?
    bool is_keep_clear_ = false;

    // 静态车辆，让lane 行驶空间不足
    bool is_lane_blocking_ = false;

    bool is_lane_change_blocking_ = false;

    // 哪些obj: 来自预测模块的定义
    bool is_caution_level_obstacle_ = false;

    double min_radius_stop_distance_ = -1.0;

    agent_drive_direction drive_dir_; 

    double time_of_headway_;
    double time_to_collision_;

    // ？
    struct ObjectTagCaseHash
    {
        size_t operator()(
                const planning::ObjectDecisionType::ObjectTagCase tag) const
        {
            return static_cast<size_t>(tag);
        }
    };

    static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                    ObjectTagCaseHash>
            s_lateral_decision_safety_sorter_;

    // ？ 定义决策优先级，且给每一个优先级赋值
    static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                    ObjectTagCaseHash>
            s_longitudinal_decision_safety_sorter_;
};

typedef IndexedList<std::string, Obstacle> IndexedObstacles;
typedef ThreadSafeIndexedList<std::string, Obstacle> ThreadSafeIndexedObstacles;

int cvt_box2d_to_polygon(Polygon2D *poly, const common::math::Box2d &box);

}  // namespace planning
}  // namespace apollo
