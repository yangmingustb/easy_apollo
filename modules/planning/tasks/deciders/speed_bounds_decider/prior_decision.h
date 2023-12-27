#pragma once


#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/common/util/common.h"

namespace apollo
{

namespace planning
{

int make_prior_object_decision(ReferenceLineInfo* const reference_line_info,
                               PathDecision* const path_decision);

bool is_ignore_for_ego_back_object_prior_decision(
        const SLBoundary& adc_sl_bound, const SLBoundary& obs_boundary,
        Obstacle* obstacle, const localization::Pose& veh_pose,
        ReferenceLineInfo* const reference_line_info);

bool is_ignore_for_ego_forward_object_prior_decision(
        const SLBoundary& adc_sl_bound, const SLBoundary& obs_boundary,
        Obstacle* obstacle, const localization::Pose& veh_pose,
        ReferenceLineInfo* const reference_line_info);

}  // namespace planning

}  // namespace apollo
