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
 *   @file
 **/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/planning/proto/task_config.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/common/math/polygon_base.h"

namespace apollo
{
namespace planning
{
// st boundary decider已经生成了boundary, 这里又计算boundary?
// st_obstacles_processor,也进行了st boudary计算。
class STBoundaryMapper
{
public:
    STBoundaryMapper(const SpeedBoundsDeciderConfig& config,
                     const ReferenceLine& reference_line,
                     const PathData& path_data, const double planning_distance,
                     const double planning_time,
                     const std::shared_ptr<DependencyInjector>& injector);

    virtual ~STBoundaryMapper() = default;

    common::Status ComputeSTBoundary(PathDecision* path_decision);


private:
    FRIEND_TEST(StBoundaryMapperTest, check_overlap_test);

    /** @brief Calls GetOverlapBoundaryPoints to get upper and lower points
     * for a given obstacle, and then formulate STBoundary based on that.
     * It also labels boundary type based on previously documented decisions.
     */
    void ComputeSTBoundary(Obstacle* obstacle) const;

    /** @brief Map the given obstacle onto the ST-Graph. The boundary is
     * represented as upper and lower points for every s of interests.
     * Note that upper_points.size() = lower_points.size()
     */
    bool GetOverlapBoundaryPoints(
            const std::vector<common::PathPoint>& path_points,
            const Obstacle& obstacle, std::vector<STPoint>* upper_points,
            std::vector<STPoint>* lower_points) const;

    /** @brief Given a path-point and an obstacle bounding box, check if the
     *        ADC, when at that path-point, will collide with the obstacle.
     * @param The path-point of the center of rear-axis for ADC.
     * @param The bounding box of the obstacle.
     * @param The extra lateral buffer for our ADC.
     */
    bool CheckOverlap(const common::PathPoint& path_point,
                      const common::math::Box2d& obs_box,
                      const double l_buffer) const;

    /** @brief Maps the closest STOP decision onto the ST-graph. This STOP
     * decision can be stopping for blocking obstacles, or can be due to
     * traffic rules, etc.
     */
    bool ComputeSTBoundaryByStopDecision(
            Obstacle* stop_obstacle, const ObjectDecisionType& decision) const;

    /** @brief Fine-tune the boundary for yielding or overtaking obstacles.
     * Increase boundary on the s-dimension or set the boundary type, etc.,
     * when necessary.
     */
    void ComputeSTBoundaryWithDecision(
            Obstacle* obstacle, const ObjectDecisionType& decision) const;

    bool has_dynamic_interaction_lon_decision(
            const ObjectDecisionType& decision) const
    {
        if (decision.has_follow() || decision.has_yield() ||
            decision.has_overtake())
        {
            return true;
        }
        return false;
    }


    int generate_adc_polygon_path(double safe_buffer,
                              const std::vector<common::PathPoint>& path_points,
                              std::vector<Polygon2D>& polygon_path)
    {
        const auto& veh_param =
                common::VehicleConfigHelper::GetConfig().vehicle_param();

        Polygon2D veh_local_poly = init_adv_box(veh_param);

        Polygon2D safe_veh_poly;

        extend_adv_box_by_width(&safe_veh_poly, safe_buffer, &veh_local_poly);

        size_t path_size = path_points.size();

        Pose2D point_pose;

        Polygon2D point_polygon;

        polygon_path.clear();

        for (size_t i = 0; i < path_size; i++)
        {
            const common::PathPoint& point = path_points[i];

            point_pose.pos.x = point.x();
            point_pose.pos.y = point.y();
            point_pose.theta = point.theta();

            cvt_local_polygon_to_global(&point_polygon, &safe_veh_poly,
                                        &point_pose);

            polygon_path.emplace_back(point_polygon);
        }

        return 0;
    }

private:
    const SpeedBoundsDeciderConfig& speed_bounds_config_;
    const ReferenceLine& reference_line_;
    const PathData& path_data_;
    const common::VehicleParam& vehicle_param_;
    const double planning_max_distance_;
    const double planning_max_time_;
    std::shared_ptr<DependencyInjector> injector_;

    std::vector<Polygon2D> adc_polygon_path_;
};

}  // namespace planning
}  // namespace apollo
