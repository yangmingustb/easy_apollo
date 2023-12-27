#pragma once

#include <chrono>
#include <iostream>
#include "modules/routing/routing.h"
#include "modules/common/status/system_state.h"

namespace apollo {
namespace routing {

class RoutingComponent
{
public:
    RoutingComponent();

    int init(std::string config_dir);

    int process(apollo::common::PointENU &start_pt,
                apollo::common::PointENU &end_pt,
                std::shared_ptr<apollo::routing::RoutingResponse> &response);

    int process(std::vector<apollo::common::PointENU> &waypoints,
                std::shared_ptr<apollo::routing::RoutingResponse> &response);

    int process(const std::shared_ptr<apollo::routing::RoutingRequest>& request,
                std::shared_ptr<apollo::routing::RoutingResponse> &response);

    // 释放
    int release()
    {
        return 0;
    }

    const std::shared_ptr<apollo::routing::RoutingRequest> &
    get_default_config_routing_request()
    {
        return _request;
    }

    int updateRoutingByMapConfig(
        std::shared_ptr<apollo::routing::RoutingResponse> routing_response,
        SystemState &module_state);

    int update_routing_by_request(
            std::shared_ptr<apollo::routing::RoutingResponse> routing_response,
            SystemState &module_state, const bool new_routing_request,
            const std::shared_ptr<routing::RoutingRequest> &request);

public:
    apollo::routing::Routing _routing;
    std::shared_ptr<apollo::routing::RoutingRequest> _request;
    // std::shared_ptr<apollo::routing::RoutingResponse> _response;

};

}  // namespace routing
}  // namespace apollo