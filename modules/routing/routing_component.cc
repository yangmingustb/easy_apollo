#include "routing.h"
#include "cyber/common/file.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/proto/poi.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/routing/routing_component.h"

namespace apollo {
namespace routing {

using namespace apollo::common;
using namespace apollo::routing;

#define record_routing_response_file (1)

RoutingComponent::RoutingComponent() {}

int RoutingComponent::init(std::string config_dir)
{
    _request = std::make_shared<RoutingRequest>();

    apollo::routing::POI poi;
    bool ret;
    ret = cyber::common::GetProtoFromASCIIFile(apollo::hdmap::EndWayPointFile(),
                                               &poi);

    if (!ret)
    {
        AERROR << "failed to load file: " << apollo::hdmap::EndWayPointFile();
    }
    else
    {
        AINFO << "read route poi success";
    }

    for (const auto &landmark : poi.landmark())
    {
        AINFO << "Refreshed point of interest: " << landmark.name();
        
        // Read xyz from point.
        for (const auto &end_point : landmark.waypoint())
        {

            // Update default end way point.
            apollo::routing::LaneWaypoint new_end_point;
            auto *pose = new_end_point.mutable_pose();
            pose->set_x(end_point.pose().x());
            pose->set_y(end_point.pose().y());
            pose->set_z(end_point.pose().z());

            *_request->add_waypoint() = new_end_point;

        }

        break;
    }

    // 初始化
    _routing.Init();
    _routing.Start();


    AINFO << "init finish";

    return 0;
}

int RoutingComponent::process(
        const std::shared_ptr<apollo::routing::RoutingRequest> &request,
        std::shared_ptr<apollo::routing::RoutingResponse> &response)
{
    _request->Clear();

    _request->CopyFrom(*request);

    response->Clear();

    // 检测
    if (!_routing.Process(request, response.get()))
    {
        return -1;
    }

#if record_routing_response_file
    std::string file = "./../data/log/routing_res.bin";
    cyber::common::SetProtoToBinaryFile(*response.get(), file);

    std::string file2 = "./../data/log/routing_res.txt";
    cyber::common::SetProtoToASCIIFile(*response.get(), file2);

#endif

    // response = _response;
    common::util::FillHeader(FLAGS_routing_node_name, response.get());

    return 0;
}

int RoutingComponent::process(apollo::common::PointENU &start_pt,
                        apollo::common::PointENU &end_pt,
                        std::shared_ptr<RoutingResponse> &response)
{
    _request->clear_waypoint();

    _request->mutable_dead_end_info()->set_dead_end_routing_type(
            routing::ROUTING_IN);

    // 起始点
    LaneWaypoint *s_way_point = _request->add_waypoint();
    PointENU *s_point_enu = s_way_point->mutable_pose();
    *s_point_enu = start_pt;

    // 结束点
    LaneWaypoint *e_way_point = _request->add_waypoint();
    PointENU *e_point_enu = e_way_point->mutable_pose();
    *e_point_enu = end_pt;

    // 检测
    if (!_routing.Process(_request, response.get()))
    {
        return -1;
    }

#if 0
    if (!record_routing_file_)
    {
        std::string file = "./../data/log/routing_res.bin";
        cyber::common::SetProtoToBinaryFile(*response.get(), file);

        std::string file2 = "./../data/log/routing_res.txt";
        cyber::common::SetProtoToASCIIFile(*response.get(), file2);

        record_routing_file_ = true;
    }

#endif

    // local_view_.routing->routing_request().waypoint().at(0).pose();
    // apollo::common::PointENU tt =
    // _response->routing_request().waypoint().at(0).pose();

    // 输出
    // response = _response;
    common::util::FillHeader(FLAGS_routing_node_name, response.get());

    return 0;
}

int RoutingComponent::process(
        std::vector<apollo::common::PointENU> &waypoints,
        std::shared_ptr<apollo::routing::RoutingResponse> &response)
{
    _request->clear_waypoint();

    _request->mutable_dead_end_info()->set_dead_end_routing_type(
            routing::ROUTING_IN);

    for (int i = 0; i < waypoints.size(); ++i)
    {
        LaneWaypoint *way_point = _request->add_waypoint();
        PointENU *point_enu = way_point->mutable_pose();
        *point_enu = waypoints[i];
    }

    // 检测
    if (!_routing.Process(_request, response.get()))
    {
        return -1;
    }

    // local_view_.routing->routing_request().waypoint().at(0).pose();
    // apollo::common::PointENU tt =
    // _response->routing_request().waypoint().at(0).pose();

    // 输出
    // response = _response;

    return 0;
}


int RoutingComponent::updateRoutingByMapConfig(
        std::shared_ptr<apollo::routing::RoutingResponse> routing_response,
        SystemState &module_state)
{
    if (routing_response->road_size() > 0)
    {
        AINFO << "route data is valid, no need to generate";
        return 0;
    }

    // generate route
    // 路由
    int ret;
    const std::shared_ptr<apollo::routing::RoutingRequest> &request_config =
            get_default_config_routing_request();

    std::shared_ptr<apollo::routing::RoutingRequest> request =
            std::make_shared<routing::RoutingRequest>((*request_config));

    for (std::size_t i = 0; i < 100; i++)
    {

        ret = process(request, routing_response);

        if (ret < 0)
        {
            SystemStateSet(module_state, ModuleName::routing,
                           ModuleStateCodeError);

            AERROR << "route generation fail";
            continue;
        }
        // get response
        else
        {
            SystemStateSet(module_state, ModuleName::routing,
                           ModuleStateCodeSuccess);

            AINFO << "route generation success";
            AINFO << "route segment size: "
                  << routing_response->road_size();
            break;
        }
    }
    return 0;
}

int RoutingComponent::update_routing_by_request(
        std::shared_ptr<apollo::routing::RoutingResponse> routing_response,
        SystemState &module_state, const bool new_routing_request,
        const std::shared_ptr<routing::RoutingRequest> &request)
{
    // generate route
    // 路由
    if (!new_routing_request)
    {
        return 0;
    }


    int ret = process(request, routing_response);

    if (ret < 0)
    {
        SystemStateSet(module_state, ModuleName::routing, ModuleStateCodeError);

        AERROR << "route generation fail";
    }
    // get response
    else
    {
        SystemStateSet(module_state, ModuleName::routing,
                       ModuleStateCodeSuccess);

        AINFO << "route generation success";
        AINFO << "route segment size: " << routing_response->road_size();
    }

    return 0;
}

}  // namespace routing
}  // namespace apollo