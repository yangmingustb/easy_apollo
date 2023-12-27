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

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo
{
namespace planning
{

enum class STLocation
{
    ABOVE = 1,
    BELOW = 2,
    CROSS = 3,
};

/**             s
 * @brief          |                   o--o--o
 * @brief          |                   |     |          <--- st boundary
 * @brief          |                   o--o--o
 * @brief          |
 * @brief          |
 * @note           ------------------------------------  time
 * @retval None
 * 一个障碍物，可能有多条预测轨迹，所以可能有多个st boundary. st
 * * 构成的polygon点是逆时针存储
 */

class STBoundary : public common::math::Polygon2d
{
public:
    /** Constructors:
     *   STBoundary must be initialized with a vector of ST-point pairs.
     *   Each pair refers to a time t, with (lower_s, upper_s).
     */
    STBoundary() = default;
    explicit STBoundary(
            const std::vector<std::pair<STPoint, STPoint>>& point_pairs,
            bool is_accurate_boundary = false);

    explicit STBoundary(const common::math::Box2d& box) = delete;
    explicit STBoundary(std::vector<common::math::Vec2d> points) = delete;

    /** @brief Wrapper of the constructor (old).
     */
    static STBoundary CreateInstance(const std::vector<STPoint>& lower_points,
                                     const std::vector<STPoint>& upper_points);

    /** @brief Wrapper of the constructor. It doesn't use RemoveRedundantPoints
     * and generates an accurate ST-boundary.
     */
    static STBoundary CreateInstanceAccurate(
            const std::vector<STPoint>& lower_points,
            const std::vector<STPoint>& upper_points);

    /** @brief Default destructor.
     */
    ~STBoundary() = default;

    bool IsEmpty() const { return lower_points_.empty(); }

    bool GetUnblockSRange(const double curr_time, double* s_upper,
                          double* s_lower) const;

    bool GetBoundarySRange(const double curr_time, double* s_upper,
                           double* s_lower) const;

    bool GetBoundarySlopes(const double curr_time, double* ds_upper,
                           double* ds_lower) const;

    // if you need to add boundary type, make sure you modify
    // GetUnblockSRange accordingly.
    // type用来干什么的？
    enum class BoundaryType
    {
        UNKNOWN,
        STOP,
        FOLLOW,
        YIELD,
        OVERTAKE,

        // ref to: KeepClear.h
        KEEP_CLEAR,
    };

    static std::string TypeName(BoundaryType type);

    BoundaryType boundary_type() const;
    const std::string& id() const;
    double characteristic_length() const;

    void set_id(const std::string& id);
    void SetBoundaryType(const BoundaryType& boundary_type);
    void SetCharacteristicLength(const double characteristic_length);

    double min_s() const;
    double min_t() const;
    double max_s() const;
    double max_t() const;

    std::vector<STPoint> upper_points() const { return upper_points_; }
    std::vector<STPoint> lower_points() const { return lower_points_; }

    // Used by st-optimizer.
    bool IsPointInBoundary(const STPoint& st_point) const;
    
    STBoundary ExpandByS(const double s) const;
    
    STBoundary ExpandLowerBoundByS(const double s) const;


    STBoundary ExpandByT(const double t) const;

    // Unused function so far.
    STBoundary CutOffByT(const double t) const;

    // Used by Lattice planners.
    STPoint upper_left_point() const;
    STPoint upper_right_point() const;
    STPoint bottom_left_point() const;
    STPoint bottom_right_point() const;

    void set_upper_left_point(STPoint st_point);
    void set_upper_right_point(STPoint st_point);
    void set_bottom_left_point(STPoint st_point);
    void set_bottom_right_point(STPoint st_point);

    void set_obstacle_road_right_ending_t(double road_right_ending_t)
    {
        obstacle_road_right_ending_t_ = road_right_ending_t;
    }
    double obstacle_road_right_ending_t() const
    {
        return obstacle_road_right_ending_t_;
    }

    bool is_end_interaction_point_valid() const
    {
       return end_interaction_point_valid_;
    }

    void set_end_interaction_point_valid(bool valid)
    {
        end_interaction_point_valid_ = valid;
    }

    void set_end_interaction_poi(double t, double s)
    {
        end_interaction_poi_.set_time(t);
        end_interaction_poi_.set_s_gap(s);
    }

    const st_gap_poi& get_end_interaction_poi() const
    {
       return end_interaction_poi_;
    }

private:
    /** @brief The sanity check function for a vector of ST-point pairs.
     */
    bool IsValid(
            const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const;

    /** @brief Returns true if point is within max_dist distance to seg.
     */
    bool IsPointNear(const common::math::LineSegment2d& seg,
                     const common::math::Vec2d& point, const double max_dist);

    /** @brief Sometimes a sequence of upper and lower points lie almost on
     * two straightlines. In this case, the intermediate points are removed,
     * with only the end-points retained.
     */
    // TODO(all): When slope is high, this may introduce significant errors.
    // Also, when accumulated for multiple t, the error can get significant.
    // This function should be reconsidered, because it may be dangerous.
    void RemoveRedundantPoints(
            std::vector<std::pair<STPoint, STPoint>>* point_pairs);
    FRIEND_TEST(StBoundaryTest, remove_redundant_points);

    /** @brief Given time t, find a segment denoted by left and right idx, that
     * contains the time t.
     * - If t is less than all or larger than all, return false.
     */
    bool GetIndexRange(const std::vector<STPoint>& points, const double t,
                       size_t* left, size_t* right) const;
    FRIEND_TEST(StBoundaryTest, get_index_range);

private:
    BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

    std::vector<STPoint> upper_points_;
    std::vector<STPoint> lower_points_;

    // obs id
    std::string id_;

    // 这是什么值：特征距离
    // if is stop, 就是在障碍物后方的停车距离，ego 车头和obs的距离
    // if is follow, 就是跟车距离，ego 车头和obs的距离
    // 这个值是正值
    // 而 ObjectFollow decision, distance s是负值,是车头和障碍物的距离。
    // message ObjectFollow
    // {
    //     optional double distance_s =
    //             1;  // minimum longitudinal distance in meters
    //     optional apollo.common.PointENU fence_point = 2;
    //     optional double fence_heading = 3;
    // }

    double characteristic_length_ = 1.0;

    // ego path 和obs path最后一个交互点，形成的s gap point, 用来计算st drive
    // boundary.
    // follow decision可能会填充；
    // yield decision可能会填充；
    bool end_interaction_point_valid_ = false;
    st_gap_poi end_interaction_poi_;

    double min_s_ = std::numeric_limits<double>::max();
    double max_s_ = std::numeric_limits<double>::lowest();

    // 下界最小时间，下界最大时间
    double min_t_ = std::numeric_limits<double>::max();
    double max_t_ = std::numeric_limits<double>::lowest();

    STPoint bottom_left_point_;
    STPoint bottom_right_point_;
    STPoint upper_left_point_;
    STPoint upper_right_point_;

    // obs拥有路权的截止时间
    // for static obs: 7秒
    // for dynamic obs： 
    // 如果自车高路权，obstacle_road_right_ending_t_=0
    // 自车低路权，obstacle_road_right_ending_t_=交互时间
    double obstacle_road_right_ending_t_;
};

std::string get_st_location_string(const STLocation& location);

}  // namespace planning
}  // namespace apollo
