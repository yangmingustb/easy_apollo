#include "modules/routing/routing.h"

using namespace apollo;
using namespace apollo::routing;

int main()
{
    FLAGS_map_dir = "./../data/hd_map";

    Routing routing;

    // 初始化
    routing.Init();
    routing.Start();

    // feed data
    RoutingRequest req;
    apollo::common::PointENU start_pt, end_pt;
    // start_pt.set_x(-7389.951648);
    // start_pt.set_y(407.580945);

    start_pt.set_x(-7398.3933004880992);
    start_pt.set_y(1362.1274828802491);

    end_pt.set_x(-9760.923282);
    end_pt.set_y(11569.733611);

    LaneWaypoint *way_point_s = req.add_waypoint();
    way_point_s->set_allocated_pose(&start_pt);

    LaneWaypoint *way_point_e = req.add_waypoint();
    way_point_e->set_allocated_pose(&end_pt);

    // 检测
    std::shared_ptr<RoutingRequest> request =
            std::make_shared<RoutingRequest>(req);
    auto response = std::make_shared<RoutingResponse>();
    if (!routing.Process(request, response.get()))
    {
        return false;
    }

    // 释放

    return 0;
}
