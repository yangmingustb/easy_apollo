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

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/proto/drive_state.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo
{
namespace planning
{

#define debug_ref_line_info (0)
#define flag_publish_ref_line_to_cyber (0)

enum class adc_direction
{
    LEFT,
    RIGHT,
    FRONT,
    BACK
};

/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */
class ReferenceLineInfo
{
public:
    enum class LaneType
    {
        LeftForward,
        LeftReverse,
        RightForward,
        RightReverse
    };
    ReferenceLineInfo() = default;

    ReferenceLineInfo(const common::VehicleState& vehicle_state,
                      const common::TrajectoryPoint& adc_planning_point,
                      const ReferenceLine& reference_line,
                      const hdmap::RouteSegments& segments);

    bool Init(const std::vector<const Obstacle*>& obstacles);

    bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
    Obstacle* AddObstacle(const Obstacle* obstacle);

    const common::VehicleState& vehicle_state() const { return vehicle_state_; }

    const double vehicle_state_heading() const
    {
        return vehicle_state_.heading();
    }

    const double get_veh_linear_velocity() const
    {
        if (vehicle_state_.has_linear_velocity())
        {
            return vehicle_state_.linear_velocity();
        }
        else
        {
            return 0;
        }
    }

    PathDecision* path_decision();
    const PathDecision& path_decision() const;

    const ReferenceLine& reference_line() const;
    ReferenceLine* mutable_reference_line();

    double SDistanceToDestination() const;
    bool ReachedDestination() const;

    void SetTrajectory(const DiscretizedTrajectory& trajectory);
    const DiscretizedTrajectory& trajectory() const;

    double Cost() const { return cost_; }
    void AddCost(double cost) { cost_ += cost; }
    void SetCost(double cost) { cost_ = cost; }

    double PriorityCost() const { return priority_cost_; }
    void SetPriorityCost(double cost) { priority_cost_ = cost; }

    // For lattice planner'speed planning target
    void SetLatticeStopPoint(const StopPoint& stop_point);

    void SetLatticeCruiseSpeed(double speed);
    
    const PlanningTarget& planning_target() const { return planning_target_; }

    void SetCruiseSpeed(double speed) { cruise_speed_ = speed; }

    double GetCruiseSpeed() const;

    void set_max_speed(double speed) { max_speed_ = speed; }

    double get_max_speed() const { return max_speed_; };

    hdmap::LaneInfoConstPtr LocateLaneInfo(const double s) const;

    bool GetNeighborLaneInfo(const ReferenceLineInfo::LaneType lane_type,
                             const double s, hdmap::Id* ptr_lane_id,
                             double* ptr_lane_width) const;

    /**
     * @brief check if current reference line is started from another reference
     *line info line. The method is to check if the start point of current
     *reference line is on previous reference line info.
     * @return returns true if current reference line starts on previous
     *reference line, otherwise false.
     **/
    bool IsStartFrom(
            const ReferenceLineInfo& previous_reference_line_info) const;

    planning_internal::Debug* mutable_debug() { return &debug_; }
    const planning_internal::Debug& debug() const { return debug_; }
    LatencyStats* mutable_latency_stats() { return &latency_stats_; }
    const LatencyStats& latency_stats() const { return latency_stats_; }

    const PathData& path_data() const;
    const PathData& fallback_path_data() const;
    const SpeedData& speed_data() const;
    PathData* mutable_path_data();
    PathData* mutable_fallback_path_data();
    SpeedData* mutable_speed_data();

    SpeedData* mutable_emergency_brake_speed_data();

    const SpeedData& emergency_brake_speed_data()
    {
        return emergency_brake_speed_data_;
    }

    const RSSInfo& rss_info() const;
    RSSInfo* mutable_rss_info();

    // aggregate final result together by some configuration
    bool CombinePathAndSpeedProfile(
            const double relative_time, const double start_s,
            DiscretizedTrajectory* discretized_trajectory);

    // adjust trajectory if it starts from cur_vehicle postion rather planning
    // init point from upstream
    bool AdjustTrajectoryWhichStartsFromCurrentPos(
            const common::TrajectoryPoint& planning_start_point,
            const std::vector<common::TrajectoryPoint>& trajectory,
            DiscretizedTrajectory* adjusted_trajectory);

    const SLBoundary& AdcSlBoundary() const;

    std::string PathSpeedDebugString() const;

    /**
     * Check if the current reference line is a change lane reference line,
     * i.e., ADC's current position is not on this reference line.
     */
    bool is_change_lane_ref_line() const;

    /**
     * Check if the current reference line is the neighbor of the vehicle
     * current position
     */
    bool IsNeighborLanePath() const;

    /**
     * Set if the vehicle can drive following this reference line
     * A planner need to set this value to true if the reference line is OK
     */
    void SetDrivable(bool drivable);
    bool IsDrivable() const;

    void ExportEngageAdvice(common::EngageAdvice* engage_advice,
                            PlanningContext* planning_context) const;

    const hdmap::RouteSegments& Lanes() const;
    std::list<hdmap::Id> TargetLaneId() const;

    // planning内部status转换成decision result
    void ExportDecision(DecisionResult* decision_result,
                        PlanningContext* planning_context) const;

    void SetJunctionRightOfWay(const double junction_s,
                               const bool is_protected) const;

    ADCTrajectory::RightOfWayStatus GetRightOfWayStatus() const;

    hdmap::Lane::LaneTurn GetPathTurnType(const double s) const;

    bool GetIntersectionRightofWayStatus(
            const hdmap::PathOverlap& pnc_junction_overlap) const;

    double OffsetToOtherReferenceLine() const
    {
        return offset_to_other_reference_line_;
    }
    void SetOffsetToOtherReferenceLine(const double offset)
    {
        offset_to_other_reference_line_ = offset;
    }

    const std::vector<PathBoundary>& GetCandidatePathBoundaries() const;

    void SetCandidatePathBoundaries(
            std::vector<PathBoundary>&& candidate_path_boundaries);

    const std::vector<PathData>& GetCandidatePathData() const;

    void SetCandidatePathData(std::vector<PathData>&& candidate_path_data);

    Obstacle* GetBlockingObstacle() const { return blocking_obstacle_; }

    Obstacle* get_mutable_close_following_obstacle()
    {
        return close_following_obstacle_;
    }

    Obstacle* get_mutable_close_stop_obstacle()
    {
        return close_stop_obstacle_;
    }

    const Obstacle* get_mutable_close_following_obstacle() const
    {
        return close_following_obstacle_;
    }

    const Obstacle* get_const_close_stop_obstacle() const
    {
        return close_stop_obstacle_;
    }

    void SetBlockingObstacle(const std::string& blocking_obstacle_id);

    void set_close_following_obstacle(const std::string& obstacle_id);

    void set_close_stop_obstacle(const std::string& obstacle_id);

    void clear_following_obstacle();

    void clear_stop_obstacle();

    bool is_path_lane_borrow() const { return is_path_lane_borrow_; }

    void set_is_path_lane_borrow(const bool is_path_lane_borrow)
    {
        is_path_lane_borrow_ = is_path_lane_borrow;
    }

    void set_is_on_reference_line() { is_on_reference_line_ = true; }

    bool is_adc_on_reference_line() const { return is_on_reference_line_; }

    uint32_t GetPriority() const { return reference_line_.GetPriority(); }

    void SetPriority(uint32_t priority)
    {
        reference_line_.SetPriority(priority);
    }

    void set_trajectory_type(
            const ADCTrajectory::TrajectoryType trajectory_type)
    {
        trajectory_type_ = trajectory_type;
    }

    ADCTrajectory::TrajectoryType trajectory_type() const
    {
        return trajectory_type_;
    }

    StGraphData* mutable_st_graph_data() { return &st_graph_data_; }

    const StGraphData& st_graph_data() { return st_graph_data_; }

    // different types of overlaps that can be handled by different scenarios.
    enum OverlapType
    {
        CLEAR_AREA = 1,
        CROSSWALK = 2,
        OBSTACLE = 3,
        PNC_JUNCTION = 4,
        SIGNAL = 5,
        STOP_SIGN = 6,
        YIELD_SIGN = 7,
    };

    const std::vector<std::pair<OverlapType, hdmap::PathOverlap>>&
    FirstEncounteredOverlaps() const
    {
        return first_encounter_overlaps_;
    }

    int GetPnCJunction(const double s,
                       hdmap::PathOverlap* pnc_junction_overlap) const;

    std::vector<common::SLPoint> GetAllStopDecisionSLPoint() const;

    void SetTurnSignal(const common::VehicleSignal::TurnSignal& turn_signal);
    void SetEmergencyLight();

    void set_path_reusable(const bool path_reusable)
    {
        path_reusable_ = path_reusable;
    }

    bool path_reusable() const { return path_reusable_; }

    double get_dist_from_plan_start_to_ref_line()
    {
        return (adc_sl_boundary_.start_l() + adc_sl_boundary_.end_l()) / 2;
    }

    hdmap::Id get_start_lane_id()
    {
      return  lanes_.at(0).lane->id();
    }

    bool has_lane_borrow_path(LaneBorrowDirection dir)
    {
        for (auto& path : candidate_path_data_)
        {
            if (dir == LaneBorrowDirection::LANE_BORROW_DIRECTION_LEFT &&
                path.path_label().find("left") != std::string::npos)
            {
                return true;
            }
            if (dir == LaneBorrowDirection::LANE_BORROW_DIRECTION_RIGHT &&
                path.path_label().find("right") != std::string::npos)
            {
                return true;
            }

        }

        return false;
    }

    int publish_ref_line_to_cyber(
            apollo::planning_internal::Debug* ptr_debug) const
    {
        if (!flag_publish_ref_line_to_cyber)
        {
            return 0;
        }
        auto* reference_line_path =
                ptr_debug->mutable_planning_data()->add_path();

        reference_line_path->set_name("reference_line");

        const auto& reference_points =
                reference_line_.reference_points();

        double s = 0.0;
        double prev_x = 0.0;
        double prev_y = 0.0;
        bool empty_path = true;

        for (const auto& reference_point : reference_points)
        {
            auto* path_point = reference_line_path->add_path_point();
            path_point->set_x(reference_point.x());
            path_point->set_y(reference_point.y());
            path_point->set_theta(reference_point.heading());
            path_point->set_kappa(reference_point.kappa());
            path_point->set_dkappa(reference_point.dkappa());
            if (empty_path)
            {
                path_point->set_s(0.0);
                empty_path = false;
            }
            else
            {
                double dx = reference_point.x() - prev_x;
                double dy = reference_point.y() - prev_y;
                s += std::hypot(dx, dy);
                path_point->set_s(s);
            }
            prev_x = reference_point.x();
            prev_y = reference_point.y();
        }

        return 0;
    }

    bool has_neighbour_lane(adc_direction direction, const double s) const;

    double get_neighbour_lane_width(adc_direction direction,
                                           const double s) const;

    int set_start_point_sl(const common::SLPoint& sl)
    {
        start_point_sl_ = sl;
        return 0;
    }

    void set_adc_point_sl(const common::SLPoint& sl)
    {
        adc_point_sl_ = sl;
        
        return;
    }

    const common::SLPoint& get_start_point_sl()
    {
        return start_point_sl_;
    }

    const common::SLPoint& get_adc_point_sl()
    {
        return adc_point_sl_;
    }

    double get_lateral_path_length()
    {
        return path_data_.discretized_path().max_s();
    }

    int set_speed_decision(const SpeedDecision& speed_decision)
    {
        speed_decision_ = speed_decision;
        return 0;
    }

    const SpeedDecision& get_speed_decision() const { return speed_decision_; }

    SpeedDecision speed_decision_;

private:
    void InitFirstOverlaps();

    bool CheckChangeLane() const;

    void SetTurnSignalBasedOnLaneTurnType(
            common::VehicleSignal* vehicle_signal) const;

    void ExportVehicleSignal(common::VehicleSignal* vehicle_signal) const;

    bool is_irrelevant_obstacle(const Obstacle& obstacle);

    void MakeDecision(DecisionResult* decision_result,
                      PlanningContext* planning_context) const;

    int MakeMainStopDecision(DecisionResult* decision_result) const;

    void MakeMainMissionCompleteDecision(
            DecisionResult* decision_result,
            PlanningContext* planning_context) const;

    void MakeEStopDecision(DecisionResult* decision_result) const;

    void SetObjectDecisions(ObjectDecisions* object_decisions) const;

    bool AddObstacleHelper(const std::shared_ptr<Obstacle>& obstacle);

    bool GetFirstOverlap(const std::vector<hdmap::PathOverlap>& path_overlaps,
                         hdmap::PathOverlap* path_overlap);



private:
    static std::unordered_map<std::string, bool> junction_right_of_way_map_;
    const common::VehicleState vehicle_state_;
    const common::TrajectoryPoint adc_planning_point_;
    ReferenceLine reference_line_;


    // 相对于reference line的sl
    common::SLPoint start_point_sl_;

    common::SLPoint adc_point_sl_;

    /**
     * @brief this is the number that measures the goodness of this reference
     * line. The lower the better. cost 由什么组成：
     */
    double cost_ = 0.0;

    // reference line生成的path,speed success，就是true. fallback 也算success.
    // path， speed要通过约束检查
    bool is_drivable_ = true;

    // st数据有3个地方存储，有什么区别：
    // 
    PathDecision path_decision_;

    // red light, stop sign 是否算block?,不算，需要是real obs。找到最近的那个obs
    // updated by selected path data
    // dynamic or static obs is blocking?
    Obstacle* blocking_obstacle_;

    // 最近的follow decision obstacle
    Obstacle* close_following_obstacle_;

    Obstacle* close_stop_obstacle_;

    std::vector<PathBoundary> candidate_path_boundaries_;

    // 一条参考线的候选路径,pwj planner生成的所有path
    // 然后，path assessment decider会删除无效的path
    // 删除原则：
    // * 偏离ref line 20米
    std::vector<PathData> candidate_path_data_;

    // 参考线的best path
    // 存储哪些数据：
    // path assesment的结果
    // 如果是path reuse, 由reuse decider填充
    PathData path_data_;
    PathData fallback_path_data_;

    // 会存储task的数据：dp -> qp --> nlp, 存在覆盖关系
    SpeedData speed_data_;

    // 紧急刹车速度曲线, 以最大制动生成，实际上就是动力学约束的下界
    // 如果当前车速为0，不需要生成这个值
    SpeedData emergency_brake_speed_data_;

    // 参考线best traj
    DiscretizedTrajectory discretized_trajectory_;

    RSSInfo rss_info_;

    /**
     * @brief SL boundary of stitching point (starting point of plan trajectory)
     * relative to the reference line
     */
    SLBoundary adc_sl_boundary_;

    //最后在哪里存储？ 会拷贝到traj, 且会发送
    planning_internal::Debug debug_;

    LatencyStats latency_stats_;

    // ref line
    hdmap::RouteSegments lanes_;

    // adc 占用了ref line车道
    bool is_on_reference_line_ = false;

    // 处于lane borrow state
    bool is_path_lane_borrow_ = false;

    ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED;

    double offset_to_other_reference_line_ = 0.0;

    double priority_cost_ = 0.0;

    // pwj path planner use this value or not?
    PlanningTarget planning_target_;

    ADCTrajectory::TrajectoryType trajectory_type_ = ADCTrajectory::UNKNOWN;

    /**
     * Overlaps encountered in the first time along the reference line in front
     * of the vehicle
     * // 记录前方场景类型
     */
    std::vector<std::pair<OverlapType, hdmap::PathOverlap>>
            first_encounter_overlaps_;

    /**
     * @brief Data generated by speed_bounds_decider for constructing st_graph
     * for different st optimizer
     * 最后不会发送给cyber rt
     */
    StGraphData st_graph_data_;

    common::VehicleSignal vehicle_signal_;

    // 速度规划的期望速度？ 该值对速度优化的影响不大，那么对DP影响如何？
    // 如果前方有障碍物，巡航速度可以是障碍物的速度
    double cruise_speed_ = 0.0;

    // 这个值就是限速，允许比ego current speed 小.
    // updated by lane borrow decider
    double max_speed_ = 0.0;

    bool path_reusable_ = false;


    DISALLOW_COPY_AND_ASSIGN(ReferenceLineInfo);
};

}  // namespace planning
}  // namespace apollo
