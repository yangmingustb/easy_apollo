#include "modules/viz2d/viz_window.h"
#include "modules/viz2d/display_config.pb.h"
#include "modules/chassis/virtual_chassis.h"
#include "modules/chassis/virtual_chassis_component.h"

#include "modules/routing/routing_component.h"
#include "modules/common/configs/vehicle_config_helper.h"

using namespace apollo;
using namespace apollo::prediction;
using namespace apollo::common;
using namespace apollo::routing;

using apollo::cyber::Rate;
using apollo::cyber::Time;

/** virutal localization
 * virtual chasis
 *
 * planner get virtual localization,virtual chasis;
 */

int main(int argc, char** argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    // double curr_time_stamp = TimeUtils::GetTimeStamp() / 1000;
    // apollo::cyber::Clock::SetMode(apollo::cyber::proto::MODE_MOCK);
    // apollo::cyber::Clock::SetNowInSeconds(curr_time_stamp);
    // todo, 将来所有数据的时钟都需要改称apollo cyber时钟

    // double history_time = curr_time_stamp;

    // module state
    SystemState module_state;

    apollo::cyber::binary::SetName("virtual_chassis");

    // init cyber framework
    apollo::cyber::Init(apollo::cyber::binary::GetName().c_str());

    SystemStateInit(module_state);


    // viz config
    viz2d_window_read_config("./../modules");

    // 全局路径规划,使用仿真时,为了先确定车辆的位置,需要读取地图配置,生成route,
    // 将route的起点作为定位信息;
    // 随着车辆运动，定位信息使用车辆的位置，然后发送出去

    RoutingComponent routing_;
    routing_.init("./../data");

    std::shared_ptr<apollo::routing::RoutingResponse> ptr_routing_response_;

    ptr_routing_response_ =
            std::make_shared<apollo::routing::RoutingResponse>();
    ptr_routing_response_->Clear();

    routing_.updateRoutingByMapConfig(ptr_routing_response_, module_state);

    AINFO << ptr_routing_response_->header().DebugString();

    // 底盘初始化，定位初始化。对于仿真而言，需要模拟定位数据、底盘数据、感知数据。
    // 而规划、控制则是根据这些数据来计算的
    apollo::localization::Pose start_point;
    if (IsModuleSuccess(module_state, ModuleName::routing))
    {
        //   set virtual localization pose
        apollo::hdmap::Id id;
        const apollo::hdmap::HDMap *hdmap_;
        hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
        apollo::hdmap::LaneInfoConstPtr lane_ptr;

        // get init pose
        const auto &road_segment = ptr_routing_response_->road(0);
        const auto &passage = road_segment.passage(0);

        id = apollo::hdmap::MakeMapId(passage.segment(0).id());

        AINFO << "hd map lane id " << id.id();

        lane_ptr = hdmap_->GetLaneById(id);

        double heading = lane_ptr->Heading(0.0);

        AINFO << "heading " << heading;

        double start_heading = heading;

        auto &lane_point = routing_._request->waypoint().at(0);

        start_point.mutable_position()->set_x(lane_point.pose().x());
        start_point.mutable_position()->set_y(lane_point.pose().y());
        start_point.mutable_position()->set_z(lane_point.pose().z());
        start_point.set_heading(start_heading);

        AINFO << "route success";
    }


    // 虚拟底盘
    virtual_chassis_interface chassis;

    double interval_time;

    // 10 ms, 保持和控制的频率一致
    interval_time = 20.0;

    common::VehicleConfig vehicle_config_;

    vehicle_config_ = apollo::common::VehicleConfigHelper::GetConfig();

    chassis.init(vehicle_config_.vehicle_param(), start_point,
                 interval_time / 1000.0);

    int ret_value;

    running_time_debug_info();

    int key_value;

    double history_time;
    double current_time;
    double delta_time;
    history_time = apollo::cyber::Time::Now().ToSecond();

    while (apollo::cyber::OK())
    {
        const apollo::RunningTimeDebug *debug = apollo::get_debug_info();

        if (debug->pause_debug.enabled)
        {
            continue;
        }


        double cur_time = apollo::cyber::Time::Now().ToSecond();
        delta_time = (cur_time - history_time) * 1000.0;

        if (delta_time < interval_time)
        {
            continue;
        }

        AINFO << "chassis frame interval (ms): " << delta_time;

        chassis.process();

        // 底盘状态,发送出去
        chassis.publish_msg();

        history_time = cur_time;
    }

    // terminate
    apollo::cyber::Clear();
    google::ShutDownCommandLineFlags();

    return 0;
}
