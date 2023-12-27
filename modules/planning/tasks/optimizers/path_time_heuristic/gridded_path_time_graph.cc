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
 * @file gridded_path_time_graph.cc
 **/

#include "modules/planning/tasks/optimizers/path_time_heuristic/gridded_path_time_graph.h"

#include <algorithm>
#include <limits>
#include <string>

#include "modules/common/proto/pnc_point.pb.h"

#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo
{
namespace planning
{
using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::util::PointFactory;

#define debug_path_time_graph (0)
#define debug_calculate_total_cost (0)
#define debug_init_cost_table (0)
#define debug_retrieve_speed_profile (0)

namespace
{
static constexpr double kDoubleEpsilon = 1.0e-6;

edge_cost_inf_type cost_inf_type;

// Continuous-time collision check using linear interpolation as closed-loop
// dynamics
bool CheckOverlapOnDpStGraph(const std::vector<const STBoundary*>& boundaries,
                             const StGraphPoint& p1, const StGraphPoint& p2)
{
    if (FLAGS_use_st_drivable_boundary)
    {
        return false;
    }
    for (const auto* boundary : boundaries)
    {
        if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR)
        {
            continue;
        }
        // Check collision between a polygon and a line segment
        if (boundary->HasOverlap({p1.point(), p2.point()}))
        {
            return true;
        }
    }
    return false;
}
}  // namespace

GriddedPathTimeGraph::GriddedPathTimeGraph(
        const StGraphData& st_graph_data,
        const DpStSpeedOptimizerConfig& dp_config,
        const std::vector<const Obstacle*>& obstacles,
        const common::TrajectoryPoint& init_point,
        const SLBoundary& adc_sl_bound) :
    st_graph_data_(st_graph_data),
    gridded_path_time_graph_config_(dp_config),
    obstacles_(obstacles),
    init_point_(init_point),
    adc_sl_bound_(adc_sl_bound),
    dp_st_cost_(dp_config, st_graph_data_.total_time_by_conf(),
                st_graph_data_.path_length(), obstacles,
                st_graph_data_.st_drivable_boundary(), init_point_,
                adc_sl_bound)
{
    total_length_t_ = st_graph_data_.total_time_by_conf();
    unit_t_ = gridded_path_time_graph_config_.unit_t();
    total_length_s_ = st_graph_data_.path_length();
    dense_unit_s_ = gridded_path_time_graph_config_.dense_unit_s();
    sparse_unit_s_ = gridded_path_time_graph_config_.sparse_unit_s();
    dense_dimension_s_ = gridded_path_time_graph_config_.dense_dimension_s();

    // 不要使用车辆限制来采样，会降低解的可能性，因为dp存在取整的现象
    // Safety approach preventing unreachable acceleration/deceleration
    // max_acceleration_ = std::min(
    //         std::abs(vehicle_param_.max_acceleration()),
    //         std::abs(gridded_path_time_graph_config_.max_acceleration()));
    max_acceleration_ = gridded_path_time_graph_config_.max_acceleration();

    max_deceleration_ =
            -1.0 * std::abs(gridded_path_time_graph_config_.max_deceleration());
}

Status GriddedPathTimeGraph::Search(SpeedData* const speed_data)
{
    static constexpr double kBounadryEpsilon = 1e-2;

    // 判断起点有obs
    for (const auto& boundary : st_graph_data_.st_boundaries())
    {
        // KeepClear obstacles not considered in Dp St decision
        if (boundary->boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR)
        {
            continue;
        }
        // If init point in collision with obstacle, return speed fallback
        if (boundary->IsPointInBoundary({0.0, 0.0}) ||
            (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
             std::fabs(boundary->min_s()) < kBounadryEpsilon))
        {
            dimension_t_ =
                    static_cast<uint32_t>(std::ceil(
                            total_length_t_ / static_cast<double>(unit_t_))) +
                    1;

            std::vector<SpeedPoint> speed_profile;
            double t = 0.0;
            for (uint32_t i = 0; i < dimension_t_; ++i, t += unit_t_)
            {
                speed_profile.push_back(PointFactory::ToSpeedPoint(0, t));
            }
            *speed_data = SpeedData(speed_profile);

            AINFO << " start point has obstacle, so no need dp for speed "
                     "search";

            return Status::OK();
        }
    }

    // st graph search, st point init
    if (!InitCostTable().ok())
    {
        const std::string msg = "Initialize cost table failed.";
        AERROR << msg;

        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // init speed limit
    if (!InitSpeedLimitLookUp().ok())
    {
        const std::string msg = "Initialize speed limit lookup table failed.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // dp搜索过程
    if (!CalculateTotalCost().ok())
    {
        const std::string msg = "Calculate total cost failed.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // 开始回溯
    if (!RetrieveSpeedProfile(speed_data).ok())
    {
        const std::string msg = "Retrieve best speed profile failed.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    return Status::OK();
}

Status GriddedPathTimeGraph::InitCostTable()
{
    // s： 2个分辨率
    // t: 一致分辨率
    // Time dimension is homogeneous while Spatial dimension has two
    // resolutions, dense and sparse with dense resolution coming first in the
    // spatial horizon

    // Sanity check for numerical stability
    if (unit_t_ < kDoubleEpsilon)
    {
        const std::string msg = "unit_t is smaller than the kDoubleEpsilon.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // Sanity check on s dimension setting
    if (dense_dimension_s_ < 1)
    {
        const std::string msg = "dense_dimension_s is at least 1.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    dimension_t_ = static_cast<uint32_t>(std::ceil(
                           total_length_t_ / static_cast<double>(unit_t_))) +
                   1;

    double sparse_length_s =
            total_length_s_ -
            static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;
    sparse_dimension_s_ =
            sparse_length_s > std::numeric_limits<double>::epsilon()
                    ? static_cast<uint32_t>(
                              std::ceil(sparse_length_s / sparse_unit_s_))
                    : 0;

    dense_dimension_s_ =
            sparse_length_s > std::numeric_limits<double>::epsilon()
                    ? dense_dimension_s_
                    : static_cast<uint32_t>(
                              std::ceil(total_length_s_ / dense_unit_s_)) +
                              1;

    dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;

    // Sanity Check
    if (dimension_t_ < 1 || dimension_s_ < 1)
    {
        const std::string msg = "Dp st cost table size incorrect.";
        AERROR << msg;

        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // cost table init
    cost_table_ = std::vector<std::vector<StGraphPoint>>(
            dimension_t_,
            std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));

    double curr_t = 0.0;
    for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_)
    {
        auto& cost_table_i = cost_table_[i];

        // 密集s
        double curr_s = 0.0;

        for (uint32_t j = 0; j < dense_dimension_s_;
             ++j, curr_s += dense_unit_s_)
        {
            cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));

#if debug_init_cost_table
            if (i == 0 || i == 1 || i == 2)
            {
                AINFO << "time index:" << i << ", s index: " << j
                      << ", s value: " << curr_s;
            }

#endif
        }

        // 稀疏s
        curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ +
                 sparse_unit_s_;

        for (uint32_t j = dense_dimension_s_; j < cost_table_i.size();
             ++j, curr_s += sparse_unit_s_)
        {
            cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));

#if debug_init_cost_table
            if (i == 0 || i == 1 || i == 2)
            {
                AINFO << "time index:" << i << ", s index: " << j
                      << ", s value: " << curr_s;
            }

#endif
        }
    }

    // s index init
    const auto& cost_table_0 = cost_table_[0];
    spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);
    for (uint32_t i = 0; i < cost_table_0.size(); ++i)
    {
        spatial_distance_by_index_[i] = cost_table_0[i].point().s();
    }

    return Status::OK();
}

Status GriddedPathTimeGraph::InitSpeedLimitLookUp()
{
    speed_limit_by_index_.clear();

    speed_limit_by_index_.resize(dimension_s_);

    const auto& speed_limit = st_graph_data_.speed_limit();

    for (uint32_t i = 0; i < dimension_s_; ++i)
    {
        speed_limit_by_index_[i] =
                speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
    }
    return Status::OK();
}

Status GriddedPathTimeGraph::CalculateTotalCost()
{
    // col and row are for STGraph
    // t corresponding to col
    // s corresponding to row

    // 确定了t，下一步搜索的s上下届
    size_t next_highest_row = 0;
    size_t next_lowest_row = 0;

    for (size_t c = 0; c < cost_table_.size(); ++c)
    {
        // 每一次搜索的s维度
        int count = static_cast<int>(next_highest_row) -
                    static_cast<int>(next_lowest_row) + 1;

        if (count > 0)
        {
            std::vector<std::future<void>> results;
            for (size_t s_idx = next_lowest_row; s_idx <= next_highest_row;
                 ++s_idx)
            {
                auto msg = std::make_shared<StGraphMessage>(c, s_idx);

                if (FLAGS_enable_multi_thread_in_dp_st_graph)
                {
                    results.push_back(cyber::Async(
                            &GriddedPathTimeGraph::CalculateCostAt, this, msg));
                }
                else
                {
                    CalculateCostAt(msg);
                    // calculate_point_cost_by_piece_wise_acc(msg);
                    // calculate_point_cost_by_piece_wise_speed(msg);

#if debug_calculate_total_cost

                    auto& cost_cr = cost_table_[c][s_idx];

                    if (std::isinf(cost_cr.total_cost()))
                    {
                        AINFO << "cost inf type: " << int(cost_inf_type);
                    }

                    AINFO << "time index:" << c << ", s index: " << s_idx
                          << ", s value: " << cost_cr.point().s()
                          << ", cost: " << cost_cr.total_cost()
                          << ", v: " << cost_cr.GetOptimalSpeed();

#endif
                }
            }

            if (FLAGS_enable_multi_thread_in_dp_st_graph)
            {
                for (auto& result : results)
                {
                    result.get();
                }
            }
        }

        // 生成下次搜索的上下届
        size_t highest_row = 0;
        size_t lowest_row = cost_table_.back().size() - 1;

        for (size_t r = next_lowest_row; r <= next_highest_row; ++r)
        {
            const auto& cost_cr = cost_table_[c][r];
            if (cost_cr.total_cost() < std::numeric_limits<double>::infinity())
            {
                size_t h_r = 0;
                size_t l_r = 0;

                // 根据极限动力学，生成上界，下界
                GetRowRange(cost_cr, &h_r, &l_r);

                highest_row = std::max(highest_row, h_r);
                lowest_row = std::min(lowest_row, l_r);

// #if debug_calculate_total_cost
//                 AINFO << "c: " << c << " row " << r
//                       << ", next s index range: " << l_r << ", " << h_r;

// #endif
            }
        }

        next_highest_row = highest_row;
        next_lowest_row = lowest_row;

#if debug_calculate_total_cost
        AINFO << "next search, time index:" << c + 1
              << ", next s index range: " << next_lowest_row << ", "
              << next_highest_row;

#endif
    }

    return Status::OK();
}

void GriddedPathTimeGraph::GetRowRange(const StGraphPoint& point,
                                       size_t* next_highest_row,
                                       size_t* next_lowest_row)
{
    double v0 = 0.0;
    // TODO(all): Record speed information in StGraphPoint and deprecate this.
    // A scaling parameter for DP range search due to the lack of accurate
    // information of the current velocity (set to 1 by default since we use
    // past 1 second's average v as approximation)

    double acc_coeff = 0.5;
    if (!point.pre_point())
    {
        v0 = init_point_.v();
    }
    else
    {
        v0 = point.GetOptimalSpeed();
    }

    const auto max_s_size = dimension_s_ - 1;
    const double t_squared = unit_t_ * unit_t_;

    // 生成上界
    const double s_upper_bound = point.point().s() + v0 * unit_t_ +
                                 acc_coeff * max_acceleration_ * t_squared;

    // 取下界，损失了加速采样值
    // 比如15.9， 取下界变成了15，然后整条曲线只能减速
    const auto next_highest_itr =
            std::lower_bound(spatial_distance_by_index_.begin(),
                             spatial_distance_by_index_.end(), s_upper_bound);
    if (next_highest_itr == spatial_distance_by_index_.end())
    {
        *next_highest_row = max_s_size;
    }
    else
    {
        *next_highest_row = std::distance(spatial_distance_by_index_.begin(),
                                          next_highest_itr);
    }

    // 下界
    const double s_lower_bound =
            point.point().s() +
            std::fmax(0.0,
                      v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared);

    // 取下界，扩大了减速能力，但是在后面acc检查时，又去掉了该采样值
    // case: 采样值，14.1米，但是取下界变成14米，
    const auto next_lowest_itr =
            std::lower_bound(spatial_distance_by_index_.begin(),
                             spatial_distance_by_index_.end(), s_lower_bound);
    if (next_lowest_itr == spatial_distance_by_index_.end())
    {
        *next_lowest_row = max_s_size;
    }
    else
    {
        *next_lowest_row = std::distance(spatial_distance_by_index_.begin(),
                                         next_lowest_itr);
    }

    return;
}


void GriddedPathTimeGraph::CalculateCostAt(
        const std::shared_ptr<StGraphMessage>& msg)
{
    const uint32_t c = msg->c;
    const uint32_t r = msg->r;
    auto& cost_cr = cost_table_[c][r];

    // obs cost
    cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));
    if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max())
    {
#if debug_calculate_total_cost
        cost_inf_type = edge_cost_inf_type::collision_with_obs;

#endif

        return;
    }

    // cost from point to plan length
    cost_cr.SetSpatialPotentialCost(
            dp_st_cost_.GetSpatialPotentialCost(cost_cr));

    const auto& cost_init = cost_table_[0][0];

    // column 0, cost为0
    if (c == 0)
    {
        if (r != 0)
        {
            AERROR << "Incorrect. Row should be 0 with col = 0. row: " << r;
        }

        cost_cr.SetTotalCost(0.0);
        cost_cr.SetOptimalSpeed(init_point_.v());

        return;
    }

    const double speed_limit = speed_limit_by_index_[r];
    const double cruise_speed = st_graph_data_.cruise_speed();
    // The mininal s to model as constant acceleration formula
    // default: 0.25 * 7 = 1.75 m

    // 0.8,小于这个值，不计算speed cost，奇怪:
    const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

    // colume 1
    // s' = v_0+a *t
    // s =s_0 +v_0 * t + 1/2 * a* t^2

    if (c == 1)
    {
        const double acc =
                2 * (cost_cr.point().s() / unit_t_ - init_point_.v()) / unit_t_;

        if (acc < max_deceleration_ || acc > max_acceleration_)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::out_of_acc_limit;

#endif
            return;
        }

        // 减速，且速度为负，而速度需要>=0，故cost为inf
        if (init_point_.v() + acc * unit_t_ < -kDoubleEpsilon &&
            cost_cr.point().s() > min_s_consider_speed)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::negative_speed;

#endif
            return;
        }

        // collision，cost inf
        if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                    cost_init))
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::collision_with_obs;

#endif
            return;
        }

        cost_cr.SetTotalCost(
                cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                cost_init.total_cost() +
                CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));

        cost_cr.SetPrePoint(cost_init);
        cost_cr.SetOptimalSpeed(init_point_.v() + acc * unit_t_);
        return;
    }

    // 前一个点
    static constexpr double kSpeedRangeBuffer = 0.20;
    const double pre_lowest_s =
            cost_cr.point().s() - FLAGS_planning_upper_speed_limit *
                                          (1 + kSpeedRangeBuffer) * unit_t_;
    const auto pre_lowest_itr =
            std::lower_bound(spatial_distance_by_index_.begin(),
                             spatial_distance_by_index_.end(), pre_lowest_s);

    // father node min s index
    uint32_t r_low = 0;
    if (pre_lowest_itr == spatial_distance_by_index_.end())
    {
        r_low = dimension_s_ - 1;
    }
    else
    {
        r_low = static_cast<uint32_t>(std::distance(
                spatial_distance_by_index_.begin(), pre_lowest_itr));
    }

    // father node size
    const uint32_t r_pre_size = r - r_low + 1;

    // father node t index
    const auto& pre_col = cost_table_[c - 1];
    double curr_speed_limit = speed_limit;

    // t =2
    if (c == 2)
    {
        for (uint32_t i = 0; i < r_pre_size; ++i)
        {
            uint32_t r_pre = r - i;

            if (std::isinf(pre_col[r_pre].total_cost()))
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::father_node_cost_is_inf;

#endif
                continue;
            }

            if (pre_col[r_pre].pre_point() == nullptr)
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::no_father_node;

#endif
                continue;
            }

            // TODO(Jiaxuan): Calculate accurate acceleration by recording speed
            // data in ST point.
            // Use curr_v = (point.s - pre_point.s) / unit_t as current v
            // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
            // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
            // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t *
            // unit_t)

            // s' = v_0+a *t
            // s =s_0 +v_0 * t + 1/2 * a* t^2

            // 加速度过滤。
            //  dp为了shrink，需要设置各种过滤方式，来加速计算
            const double curr_a =
                    2 *
                    ((cost_cr.point().s() - pre_col[r_pre].point().s()) /
                             unit_t_ -
                     pre_col[r_pre].GetOptimalSpeed()) /
                    unit_t_;

            if (curr_a < max_deceleration_ || curr_a > max_acceleration_)
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::out_of_acc_limit;

#endif
                continue;
            }

            // 速度为负，cost 就inf?
            if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ <
                        -kDoubleEpsilon &&
                cost_cr.point().s() > min_s_consider_speed)
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::negative_speed;

#endif
                continue;
            }

            // Filter out continuous-time node connection which is in collision
            // with obstacle
            // if collision, 过滤计算
            if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                        pre_col[r_pre]))
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::collision_with_obs;

#endif
                continue;
            }

            curr_speed_limit =
                    std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

            const double tmp_cost =
                    cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                    pre_col[r_pre].total_cost() +
                    CalculateEdgeCostForThirdCol(r, r_pre, curr_speed_limit,
                                                 cruise_speed);

            if (tmp_cost < cost_cr.total_cost())
            {
                cost_cr.SetTotalCost(tmp_cost);
                cost_cr.SetPrePoint(pre_col[r_pre]);
                cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                                        curr_a * unit_t_);
            }
        }

        return;
    }

#if debug_calculate_total_cost
    AINFO << "pre s node size: " <<r_pre_size;

#endif

    // t >= 3
    for (uint32_t i = 0; i < r_pre_size; ++i)
    {
        uint32_t r_pre = r - i;

        if (std::isinf(pre_col[r_pre].total_cost()) ||
            pre_col[r_pre].pre_point() == nullptr)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::no_father_node;

#endif
            continue;
        }

        // Use curr_v = (point.s - pre_point.s) / unit_t as current v
        // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
        // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
        // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)

        // s'' = a
        // s' = v_0+a *t
        // s =s_0 +v_0 * t + 1/2 * a* t^2

        const double curr_a =
                2 *
                ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
                 pre_col[r_pre].GetOptimalSpeed()) /
                unit_t_;

        if (curr_a > max_acceleration_ || curr_a < max_deceleration_)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::out_of_acc_limit;

#endif
            continue;
        }

        if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ <
                    -kDoubleEpsilon &&
            cost_cr.point().s() > min_s_consider_speed)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::negative_speed;

#endif
            continue;
        }

        if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                    pre_col[r_pre]))
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::collision_with_obs;

#endif
            continue;
        }

        uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
        const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];

        if (std::isinf(prepre_graph_point.total_cost()))
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::father_node_cost_is_inf;

#endif
            continue;
        }

        if (!prepre_graph_point.pre_point())
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::no_father_node;

#endif
            continue;
        }

        const STPoint& triple_pre_point =
                prepre_graph_point.pre_point()->point();
        const STPoint& prepre_point = prepre_graph_point.point();
        const STPoint& pre_point = pre_col[r_pre].point();
        const STPoint& curr_point = cost_cr.point();

        curr_speed_limit =
                std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

        double tmp_cost =
                cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                pre_col[r_pre].total_cost() +
                CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                  curr_point, curr_speed_limit, cruise_speed);

        if (tmp_cost < cost_cr.total_cost())
        {
            cost_cr.SetTotalCost(tmp_cost);
            cost_cr.SetPrePoint(pre_col[r_pre]);
            cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                                    curr_a * unit_t_);
        }
    }

    return;
}

void GriddedPathTimeGraph::calculate_point_cost_by_piece_wise_acc(
        const std::shared_ptr<StGraphMessage>& msg)
{
    const uint32_t c = msg->c;
    const uint32_t r = msg->r;
    auto& cost_cr = cost_table_[c][r];

    // obs cost
    cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));
    if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max())
    {
#if debug_calculate_total_cost
        cost_inf_type = edge_cost_inf_type::collision_with_obs;

#endif

        return;
    }

    // cost from point to plan length
    cost_cr.SetSpatialPotentialCost(
            dp_st_cost_.GetSpatialPotentialCost(cost_cr));

    const auto& cost_init = cost_table_[0][0];

    // column 0, cost为0
    if (c == 0)
    {
        if (r != 0)
        {
            AERROR << "Incorrect. Row should be 0 with col = 0. row: " << r;
        }

        cost_cr.SetTotalCost(0.0);
        cost_cr.SetOptimalSpeed(init_point_.v());
        cost_cr.set_optimal_acc(init_point_.a());

        return;
    }

    // 接下来计算speed,acc,jerk cost

    const double speed_limit = speed_limit_by_index_[r];
    const double cruise_speed = st_graph_data_.cruise_speed();
    // The mininal s to model as constant acceleration formula
    // default: 0.25 * 7 = 1.75 m

    // 0.8,小于这个值，不计算speed cost，奇怪:
    const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

    // colume 1
    // s' = v_0+a *t
    // s =s_0 +v_0 * t + 1/2 * a* t^2

    double cur_point_acc;
    double cur_point_v;
    double prev_point_v;
    double prev_point_acc;

    if (c == 1)
    {
        cur_point_acc =
                2 * (cost_cr.point().s() / unit_t_ - init_point_.v()) / unit_t_;

        if (cur_point_acc < max_deceleration_ ||
            cur_point_acc > max_acceleration_)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::out_of_acc_limit;

#endif
            return;
        }

        // 速度为负，而速度需要>=0，故cost为inf
        cur_point_v = init_point_.v() + cur_point_acc * unit_t_;
        if (cur_point_v < -kDoubleEpsilon &&
            cost_cr.point().s() > min_s_consider_speed)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::negative_speed;

#endif
            return;
        }

        // collision，cost inf
        if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                    cost_init))
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::collision_with_obs;

#endif
            return;
        }

        cost_cr.SetTotalCost(
                cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                cost_init.total_cost() +
                calculate_edge_cost_for_second_col(r, speed_limit, cruise_speed,
                                                   cur_point_v, cur_point_acc));

        cost_cr.SetPrePoint(cost_init);
        cost_cr.SetOptimalSpeed(cur_point_v);
        cost_cr.set_optimal_acc(cur_point_acc);
        return;
    }

    // 前一个点
    static constexpr double kSpeedRangeBuffer = 0.20;
    const double pre_lowest_s =
            cost_cr.point().s() - FLAGS_planning_upper_speed_limit *
                                          (1 + kSpeedRangeBuffer) * unit_t_;
    const auto pre_lowest_itr =
            std::lower_bound(spatial_distance_by_index_.begin(),
                             spatial_distance_by_index_.end(), pre_lowest_s);

    // 确定前一个点的最小s
    // father node min s index
    uint32_t r_low = 0;
    if (pre_lowest_itr == spatial_distance_by_index_.end())
    {
        r_low = dimension_s_ - 1;
    }
    else
    {
        r_low = static_cast<uint32_t>(std::distance(
                spatial_distance_by_index_.begin(), pre_lowest_itr));
    }

    // 从father node 到当前node，可以搜索的s size
    // 需要遍历每一个father node，得到current node minimum cost
    const uint32_t r_pre_size = r - r_low + 1;

    // father node t index
    const auto& pre_col = cost_table_[c - 1];
    double curr_speed_limit = speed_limit;

    // t =2
    if (c == 2)
    {
        uint32_t r_pre;
        // 对于一个确定的当前点，遍历father node，得到current node minimum cost

        for (uint32_t i = 0; i < r_pre_size; ++i)
        {
            // 前一个点索引
            r_pre = r - i;

            if (std::isinf(pre_col[r_pre].total_cost()) ||
                pre_col[r_pre].pre_point() == nullptr)
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::no_father_node;

#endif
                continue;
            }

            // TODO(Jiaxuan): Calculate accurate acceleration by recording speed
            // data in ST point.
            // Use curr_v = (point.s - pre_point.s) / unit_t as current v
            // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
            // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
            // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t *
            // unit_t)

            // s' = v_0+a *t
            // s =s_0 +v_0 * t + 1/2 * a* t^2

            // 加速度过滤。
            //  dp为了shrink，需要设置各种过滤方式，来加速计算
            prev_point_v = pre_col[r_pre].GetOptimalSpeed();
            cur_point_acc =
                    ((cost_cr.point().s() - pre_col[r_pre].point().s()) /
                             unit_t_ -
                     prev_point_v) /
                    unit_t_ * 2;

            if (cur_point_acc < max_deceleration_ ||
                cur_point_acc > max_acceleration_)
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::out_of_acc_limit;

#endif
                continue;
            }

            // 速度为负，cost 就inf?
            cur_point_v = prev_point_v + cur_point_acc * unit_t_;
            if (cur_point_v < -kDoubleEpsilon &&
                cost_cr.point().s() > min_s_consider_speed)
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::negative_speed;

#endif
                continue;
            }

            // Filter out continuous-time node connection which is in collision
            // with obstacle
            // if collision, 过滤计算
            if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                        pre_col[r_pre]))
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::collision_with_obs;

#endif
                continue;
            }

            // get speed limit
            curr_speed_limit =
                    std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

            const double tmp_cost =
                    cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                    pre_col[r_pre].total_cost() +
                    calculate_edge_cost_for_third_col(
                            r, r_pre, curr_speed_limit, cruise_speed,
                            cur_point_v, cur_point_acc);

            if (tmp_cost < cost_cr.total_cost())
            {
                cost_cr.SetTotalCost(tmp_cost);
                cost_cr.SetPrePoint(pre_col[r_pre]);
                cost_cr.SetOptimalSpeed(cur_point_v);
                cost_cr.set_optimal_acc(cur_point_acc);
            }
        }

        return;
    }

#if debug_calculate_total_cost
    AINFO << "cur node s index: " << r << " ,father node size: " << r_pre_size;

#endif

    // t >= 3

    for (uint32_t i = 0; i < r_pre_size; ++i)
    {
        uint32_t r_pre = r - i;

        if (std::isinf(pre_col[r_pre].total_cost()) ||
            pre_col[r_pre].pre_point() == nullptr)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::no_father_node;

            // AINFO << "no_father_node";

#endif
            continue;
        }

        // Use curr_v = (point.s - pre_point.s) / unit_t as current v
        // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
        // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
        // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)

        // s'' = a
        // s' = v_0+a *t
        // s =s_0 +v_0 * t + 1/2 * a* t^2

        prev_point_v = pre_col[r_pre].GetOptimalSpeed();

        cur_point_acc =
                2 *
                ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ -
                 prev_point_v) /
                unit_t_;

        // away from acc constaints
        if (cur_point_acc > max_acceleration_ ||
            cur_point_acc < max_deceleration_)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::out_of_acc_limit;

            // AINFO << "out_of_acc_limit";

#endif
            continue;
        }

        // 速度为负
        cur_point_v = prev_point_v + cur_point_acc * unit_t_;

        if (cur_point_v < -kDoubleEpsilon &&
            cost_cr.point().s() > min_s_consider_speed)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::negative_speed;

            // AINFO << "negative_speed, father node index " << r_pre
            //       << ", curr_a " << curr_a << " last v "
            //       << pre_col[r_pre].GetOptimalSpeed();

#endif
            continue;
        }

        // if collision
        if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                    pre_col[r_pre]))
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::collision_with_obs;

            AINFO << "collision_with_obs";

#endif
            continue;
        }

        const STPoint& pre_point = pre_col[r_pre].point();
        const STPoint& curr_point = cost_cr.point();

        prev_point_acc = pre_col[r_pre].get_optimal_acc();

        curr_speed_limit =
                std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

        double tmp_cost =
                cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                pre_col[r_pre].total_cost() +
                calculate_edge_cost(pre_point, curr_point, curr_speed_limit,
                                    cruise_speed, cur_point_v, cur_point_acc,
                                    prev_point_acc);

        if (tmp_cost < cost_cr.total_cost())
        {
            cost_cr.SetTotalCost(tmp_cost);
            cost_cr.SetPrePoint(pre_col[r_pre]);
            cost_cr.SetOptimalSpeed(cur_point_v);
            cost_cr.set_optimal_acc(cur_point_acc);
        }

// #if debug_calculate_total_cost
//         AINFO << "last pt s index " << r_pre << " last pt v "
//               << pre_col[r_pre].GetOptimalSpeed() << " curr_a " << curr_a
//               << ", cur_speed " << cur_speed << ", tmp_cost " << tmp_cost;

//         if (cost_cr.pre_point() != nullptr)
//         {
//             AINFO << "minimum cost, father node "
//                   << cost_cr.pre_point()->index_s();
//         }

// #endif
    }

    return;
}

void GriddedPathTimeGraph::calculate_point_cost_by_piece_wise_speed(
        const std::shared_ptr<StGraphMessage>& msg)
{
    const uint32_t c = msg->c;
    const uint32_t r = msg->r;
    auto& cost_cr = cost_table_[c][r];

    // obs cost
    cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));
    if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max())
    {
#if debug_calculate_total_cost
        cost_inf_type = edge_cost_inf_type::collision_with_obs;

#endif

        return;
    }

    // cost from point to plan length
    cost_cr.SetSpatialPotentialCost(
            dp_st_cost_.GetSpatialPotentialCost(cost_cr));

    const auto& cost_init = cost_table_[0][0];

    // column 0, cost为0
    if (c == 0)
    {
        if (r != 0)
        {
            AERROR << "Incorrect. Row should be 0 with col = 0. row: " << r;
        }

        cost_cr.SetTotalCost(0.0);
        cost_cr.SetOptimalSpeed(init_point_.v());

        return;
    }

    // 接下来计算speed,acc,jerk cost

    const double speed_limit = speed_limit_by_index_[r];
    const double cruise_speed = st_graph_data_.cruise_speed();
    // The mininal s to model as constant acceleration formula
    // default: 0.25 * 7 = 1.75 m

    // 0.8,小于这个值，不计算speed cost，奇怪:
    const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

    double cur_pt_v;
    double prev_pt_v;
    double cur_pt_acc;

    // colume 1
    // s' = v_0+a *t
    // s =s_0 +v_0 * t + 1/2 * a* t^2

    if (c == 1)
    {
        cur_pt_v = cost_cr.point().s() / unit_t_;

        cur_pt_acc = (cur_pt_v - init_point_.v()) / unit_t_;

        if (cur_pt_acc < max_deceleration_ || cur_pt_acc > max_acceleration_)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::out_of_acc_limit;

#endif
            return;
        }

        // 速度为负，而速度需要>=0，故cost为inf
        if (cur_pt_v < -kDoubleEpsilon &&
            cost_cr.point().s() > min_s_consider_speed)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::negative_speed;

#endif
            return;
        }

        // collision，cost inf
        if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                    cost_init))
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::collision_with_obs;

#endif
            return;
        }

        cost_cr.SetTotalCost(
                cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                cost_init.total_cost() +
                CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));

        cost_cr.SetPrePoint(cost_init);
        cost_cr.SetOptimalSpeed(cur_pt_v);
        return;
    }

    // 前一个点
    static constexpr double kSpeedRangeBuffer = 0.20;
    const double pre_lowest_s =
            cost_cr.point().s() - FLAGS_planning_upper_speed_limit *
                                          (1 + kSpeedRangeBuffer) * unit_t_;
    const auto pre_lowest_itr =
            std::lower_bound(spatial_distance_by_index_.begin(),
                             spatial_distance_by_index_.end(), pre_lowest_s);

    // 确定前一个点的最小s
    // father node min s index
    uint32_t r_low = 0;
    if (pre_lowest_itr == spatial_distance_by_index_.end())
    {
        r_low = dimension_s_ - 1;
    }
    else
    {
        r_low = static_cast<uint32_t>(std::distance(
                spatial_distance_by_index_.begin(), pre_lowest_itr));
    }

    // 从father node 到当前node，可以搜索的s size
    // 需要遍历每一个father node，得到current node minimum cost
    const uint32_t r_pre_size = r - r_low + 1;

    // father node t index
    const auto& pre_col = cost_table_[c - 1];
    double curr_speed_limit = speed_limit;


    // t =2
    if (c == 2)
    {
        uint32_t r_pre;
        // 对于一个确定的当前点，遍历father node，得到current node minimum cost

        for (uint32_t i = 0; i < r_pre_size; ++i)
        {
            // 前一个点索引
            r_pre = r - i;

            if (std::isinf(pre_col[r_pre].total_cost()) ||
                pre_col[r_pre].pre_point() == nullptr)
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::no_father_node;

#endif
                continue;
            }

            // TODO(Jiaxuan): Calculate accurate acceleration by recording speed
            // data in ST point.
            // Use curr_v = (point.s - pre_point.s) / unit_t as current v
            // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
            // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
            // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t *
            // unit_t)

            // s' = v_0+a *t
            // s =s_0 +v_0 * t + 1/2 * a* t^2

            // 加速度过滤。
            //  dp为了shrink，需要设置各种过滤方式，来加速计算

            cur_pt_v = (cost_cr.point().s() - pre_col[r_pre].point().s()) /
                       unit_t_;

            cur_pt_acc =
                    (cur_pt_v - pre_col[r_pre].GetOptimalSpeed()) / unit_t_;

            if (cur_pt_acc < max_deceleration_ || cur_pt_acc > max_acceleration_)
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::out_of_acc_limit;

#endif
                continue;
            }

            // 速度为负，cost 就inf?
            if (cur_pt_v < -kDoubleEpsilon &&
                cost_cr.point().s() > min_s_consider_speed)
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::negative_speed;

#endif
                continue;
            }

            // Filter out continuous-time node connection which is in collision
            // with obstacle
            // if collision, 过滤计算
            if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                        pre_col[r_pre]))
            {
#if debug_calculate_total_cost
                cost_inf_type = edge_cost_inf_type::collision_with_obs;

#endif
                continue;
            }

            // get speed limit
            curr_speed_limit =
                    std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

            const double tmp_cost =
                    cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                    pre_col[r_pre].total_cost() +
                    CalculateEdgeCostForThirdCol(r, r_pre, curr_speed_limit,
                                                 cruise_speed);

            if (tmp_cost < cost_cr.total_cost())
            {
                cost_cr.SetTotalCost(tmp_cost);
                cost_cr.SetPrePoint(pre_col[r_pre]);
                cost_cr.SetOptimalSpeed(cur_pt_v);
            }
        }

        return;
    }

#if debug_calculate_total_cost
    AINFO << "cur node s index: " << r << " ,father node size: " << r_pre_size;

#endif

    // t >= 3

    for (uint32_t i = 0; i < r_pre_size; ++i)
    {
        uint32_t r_pre = r - i;

        if (std::isinf(pre_col[r_pre].total_cost()) ||
            pre_col[r_pre].pre_point() == nullptr)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::no_father_node;

            // AINFO << "no_father_node";

#endif
            continue;
        }

        // Use curr_v = (point.s - pre_point.s) / unit_t as current v
        // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
        // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
        // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)

        // s'' = a
        // s' = v_0+a *t
        // s =s_0 +v_0 * t + 1/2 * a* t^2

        cur_pt_v = (cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_;

        cur_pt_acc = (cur_pt_v - pre_col[r_pre].GetOptimalSpeed()) / unit_t_;

        // away from acc constaints
        if (cur_pt_acc > max_acceleration_ || cur_pt_acc < max_deceleration_)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::out_of_acc_limit;

            // AINFO << "out_of_acc_limit";

#endif
            continue;
        }

        // 速度为负
        if (cur_pt_v < -kDoubleEpsilon &&
            cost_cr.point().s() > min_s_consider_speed)
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::negative_speed;

            // AINFO << "negative_speed, father node index " << r_pre
            //       << ", curr_a " << curr_a << " last v "
            //       << pre_col[r_pre].GetOptimalSpeed();

#endif
            continue;
        }

        // if collision
        if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                    pre_col[r_pre]))
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::collision_with_obs;

            AINFO << "collision_with_obs";

#endif
            continue;
        }

        uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
        const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];

        if (std::isinf(prepre_graph_point.total_cost()))
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::father_node_cost_is_inf;


            // AINFO << "father_node_cost_is_inf";

#endif
            continue;
        }

        if (!prepre_graph_point.pre_point())
        {
#if debug_calculate_total_cost
            cost_inf_type = edge_cost_inf_type::no_father_node;

            // AINFO << "no_father_node";

#endif
            continue;
        }

        const STPoint& triple_pre_point =
                prepre_graph_point.pre_point()->point();
        const STPoint& prepre_point = prepre_graph_point.point();
        const STPoint& pre_point = pre_col[r_pre].point();
        const STPoint& curr_point = cost_cr.point();

        curr_speed_limit =
                std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);

        double tmp_cost =
                cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                pre_col[r_pre].total_cost() +
                CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                  curr_point, curr_speed_limit, cruise_speed);

        if (tmp_cost < cost_cr.total_cost())
        {
            cost_cr.SetTotalCost(tmp_cost);
            cost_cr.SetPrePoint(pre_col[r_pre]);
            cost_cr.SetOptimalSpeed(cur_pt_v);
        }

// #if debug_calculate_total_cost
//         AINFO << "last pt s index " << r_pre << " last pt v "
//               << pre_col[r_pre].GetOptimalSpeed() << " curr_a " << curr_a
//               << ", cur_speed " << cur_speed << ", tmp_cost " << tmp_cost;

//         if (cost_cr.pre_point() != nullptr)
//         {
//             AINFO << "minimum cost, father node "
//                   << cost_cr.pre_point()->index_s();
//         }

// #endif
    }

    return;
}

Status GriddedPathTimeGraph::RetrieveSpeedProfile(SpeedData* const speed_data)
{
    double min_cost = std::numeric_limits<double>::infinity();
    const StGraphPoint* best_end_point = nullptr;

    // 搜索最后一列
    for (const StGraphPoint& cur_point : cost_table_.back())
    {
        if (!std::isinf(cur_point.total_cost()) &&
            cur_point.total_cost() < min_cost)
        {
            best_end_point = &cur_point;
            min_cost = cur_point.total_cost();
        }
    }

    // 搜索最上面一行，s最大的一行
    for (const auto& row : cost_table_)
    {
        const StGraphPoint& cur_point = row.back();

        if (!std::isinf(cur_point.total_cost()) &&
            cur_point.total_cost() < min_cost)
        {
            best_end_point = &cur_point;
            min_cost = cur_point.total_cost();
        }
    }

    // 搜索结果都是infinity， 这种case在什么情况下出现：
    if (best_end_point == nullptr)
    {
        const std::string msg = "Fail to find the best feasible trajectory.";
        AERROR << msg;

        return Status::OK();
    }

#if debug_retrieve_speed_profile
    if (best_end_point != nullptr)
    {
        AINFO << "index t " << best_end_point->index_t() << " ,index s "
              << best_end_point->index_s() << " ,t "
              << best_end_point->point().t() << " ,s "
              << best_end_point->point().s() << " ,cost " << min_cost;
    }

#endif

    // 回溯
    std::vector<SpeedPoint> speed_profile;
    const StGraphPoint* cur_point = best_end_point;
    SpeedPoint speed_point;

    while (cur_point != nullptr)
    {
#if debug_retrieve_speed_profile
        AINFO << "Time: " << cur_point->point().t()
              << ", S: " << cur_point->point().s()
              << ", V: " << cur_point->GetOptimalSpeed();

#endif

        speed_point.set_s(cur_point->point().s());
        speed_point.set_t(cur_point->point().t());
        speed_profile.push_back(speed_point);

        cur_point = cur_point->pre_point();


    }

    // 速度曲线反向
    std::reverse(speed_profile.begin(), speed_profile.end());

    // 第一个点是起点，s=0.0, t=0.0。如果不是，说明内部出了错误
    double kEpsilon = std::numeric_limits<double>::epsilon();
    if (speed_profile.front().t() > kEpsilon ||
        speed_profile.front().s() > kEpsilon)
    {
        const std::string msg = "Fail to retrieve speed profile.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // 2个点之间，可能是匀速、加速、减速、静止运动，也可能是各种运动的组合。所以为了简化计算,
    // 可以计算平均速度
    double v;
    double delta_time;
    size_t speed_profile_size = speed_profile.size();

    double acc;
    double jerk;

    if (speed_profile_size > 0)
    {
        speed_profile[0].set_v(init_point_.v());
        speed_profile[0].set_a(init_point_.a());
        speed_profile[0].set_da(0.0);
    }

    for (size_t i = 1; i < speed_profile_size; ++i)
    {
        // speed
        if (i == 1)
        {
            acc = 2 * (speed_profile[i].s() / unit_t_ - init_point_.v()) /
                  unit_t_;

            v = init_point_.v() + acc * unit_t_;

            jerk = (acc - init_point_.a()) / unit_t_;

        }
        else if (i == 2)
        {
            // s' = v_0+a *t
            // s =s_0 +v_0 * t + 1/2 * a* t^2

            // 加速度过滤。
            //  dp为了shrink，需要设置各种过滤方式，来加速计算
            acc = 2 *
                  ((speed_profile[i].s() - speed_profile[i - 1].s()) / unit_t_ -
                   speed_profile[i - 1].v()) /
                  unit_t_;

            v = speed_profile[i - 1].v() + acc * unit_t_;

            jerk = (acc - speed_profile[i - 1].a()) / unit_t_;

        }
        else
        {
            // s'' = a
            // s' = v_0+a *t
            // s =s_0 +v_0 * t + 1/2 * a* t^2

            acc = 2 *
                  ((speed_profile[i].s() - speed_profile[i - 1].s()) / unit_t_ -
                   speed_profile[i - 1].v()) /
                  unit_t_;

            v = speed_profile[i - 1].v() + acc * unit_t_;

            jerk = (acc - speed_profile[i - 1].a()) / unit_t_;


        }

        speed_profile[i].set_v(v);

        speed_profile[i].set_a(acc);

        speed_profile[i].set_da(jerk);

#if debug_retrieve_speed_profile
        AINFO << "Time: " << speed_profile[i].t()
              << " , s: " << speed_profile[i].s()
              << " , v: " << speed_profile[i].v()
              << " , a: " << speed_profile[i].a()
              << " , jerk: " << speed_profile[i].da();

#endif
    }

    // fill end point
    if (speed_profile_size == 1)
    {
        speed_profile[0].set_v(init_point_.v());
        speed_profile[0].set_a(init_point_.a());
        speed_profile[0].set_da(0.0);
    }

    // copy data
    *speed_data = SpeedData(speed_profile);

    return Status::OK();
}

double GriddedPathTimeGraph::CalculateEdgeCost(const STPoint& first,
                                               const STPoint& second,
                                               const STPoint& third,
                                               const STPoint& forth,
                                               const double speed_limit,
                                               const double cruise_speed)
{
    return dp_st_cost_.GetSpeedCost(third, forth, speed_limit, cruise_speed) +
           dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
           dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}


double GriddedPathTimeGraph::CalculateEdgeCostForSecondCol(
        const uint32_t row, const double speed_limit, const double cruise_speed)
{
    double init_speed = init_point_.v();
    double init_acc = init_point_.a();
    const STPoint& pre_point = cost_table_[0][0].point();
    const STPoint& curr_point = cost_table_[1][row].point();

    return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit,
                                    cruise_speed) +
           dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                               curr_point) +
           dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                              curr_point);
}

double GriddedPathTimeGraph::calculate_edge_cost_for_second_col(
        const uint32_t row, const double speed_limit, const double cruise_speed,
        const double cur_v, const double cur_acc)
{
    double init_speed = init_point_.v();
    double init_acc = init_point_.a();
    const STPoint& pre_point = cost_table_[0][0].point();
    const STPoint& curr_point = cost_table_[1][row].point();

    double jerk;

    jerk = (cur_acc - init_acc) / unit_t_;

    return dp_st_cost_.get_speed_cost(cur_v, curr_point, pre_point, speed_limit,
                                      cruise_speed) +
           dp_st_cost_.GetAccelCost(cur_acc) + dp_st_cost_.JerkCost(jerk);
}

double GriddedPathTimeGraph::CalculateEdgeCostForThirdCol(
        const uint32_t curr_row, const uint32_t pre_row,
        const double speed_limit, const double cruise_speed)
{
    double init_speed = init_point_.v();
    const STPoint& first = cost_table_[0][0].point();
    const STPoint& second = cost_table_[1][pre_row].point();
    const STPoint& third = cost_table_[2][curr_row].point();

    return dp_st_cost_.GetSpeedCost(second, third, speed_limit, cruise_speed) +
           dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
           dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second,
                                                third);
}

double GriddedPathTimeGraph::calculate_edge_cost_for_third_col(
        const uint32_t curr_row, const uint32_t pre_row,
        const double speed_limit, const double cruise_speed, const double cur_v,
        const double cur_acc)
{
    double init_speed = init_point_.v();
    const STPoint& first = cost_table_[0][0].point();
    const STPoint& second = cost_table_[1][pre_row].point();
    const STPoint& third = cost_table_[2][curr_row].point();

    double jerk;

    jerk = (cur_acc - cost_table_[1][pre_row].get_optimal_acc()) / unit_t_;

    return dp_st_cost_.get_speed_cost(cur_v, second, third, speed_limit,
                                      cruise_speed) +
           dp_st_cost_.GetAccelCost(cur_acc) + dp_st_cost_.JerkCost(jerk);
}

double GriddedPathTimeGraph::calculate_edge_cost(
        const STPoint& third, const STPoint& forth, const double speed_limit,
        const double cruise_speed, const double cur_v, const double cur_acc,
        const double prev_point_acc)
{
    double jerk;

    jerk = (cur_acc - prev_point_acc) / unit_t_;

    return dp_st_cost_.get_speed_cost(cur_v, third, forth, speed_limit,
                                      cruise_speed) +
           dp_st_cost_.GetAccelCost(cur_acc) + dp_st_cost_.JerkCost(jerk);
}

}  // namespace planning
}  // namespace apollo
