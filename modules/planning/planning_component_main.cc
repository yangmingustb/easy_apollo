#include "modules/routing/routing_component.h"
#include "modules/planning/planning_component.h"
#include "modules/common/status/system_state.h"
#include "modules/prediction/prediction_component.h"
#include "modules/viz2d/display_config.pb.h"

using namespace apollo;
using namespace apollo::prediction;
using namespace apollo::common;
using namespace apollo::routing;
using namespace apollo::planning;


int main(int argc, char** argv)
{
    // double curr_time_stamp = TimeUtils::GetTimeStamp() / 1000;
    // apollo::cyber::Clock::SetMode(apollo::cyber::proto::MODE_MOCK);
    // apollo::cyber::Clock::SetNowInSeconds(curr_time_stamp);
    // todo, 将来所有数据的时钟都需要改称apollo cyber时钟

    // double history_time = curr_time_stamp;

    google::ParseCommandLineFlags(&argc, &argv, true);

    // module state
    SystemState module_state;
    SystemStateInit(module_state);

    apollo::cyber::binary::SetName("planning");

    // init cyber framework
    apollo::cyber::Init(apollo::cyber::binary::GetName().c_str());


    FLAGS_vehicle_config_path =
            "./../modules/common/data/vehicle_param.pb.txt";

    FLAGS_vehicle_model_config_filename = "./../modules/common/vehicle_model/conf/vehicle_model_config.pb.txt";

    // 预测初始化
    PredictionComponent prediction;

    prediction.Init();

    std::shared_ptr<apollo::storytelling::Stories> storytelling_msg;

    storytelling_msg = std::make_shared<storytelling::Stories>();

    // 全局路径规划
    // 放在预测模块后, 放在前会导致routing无结果, 可能与地图相关 TODO
    RoutingComponent routing;
    routing.init("./../modules");

    std::shared_ptr<apollo::routing::RoutingResponse> ptr_routing_response_;

    ptr_routing_response_ =
            std::make_shared<apollo::routing::RoutingResponse>();
    ptr_routing_response_->Clear();

    // 局部规划初始化
    PlanningComponent planning;
    planning.init("./../modules");

    common::VehicleConfig vehicle_config_;

    vehicle_config_ = apollo::common::VehicleConfigHelper::GetConfig();

    double steering_wheel_ratio =
            vehicle_config_.vehicle_param().steer_ratio();


    int ret_value;

    running_time_debug_info();

    routing.updateRoutingByMapConfig(ptr_routing_response_, module_state);

    AINFO << ptr_routing_response_->header().DebugString();

    int locolization_ret = 0;


    int n_planning = 0;

    int key_value;

    double history_time;
    double current_time;
    double delta_time;
    history_time = apollo::cyber::Time::Now().ToSecond();

    double planning_start_time;
    double planning_end_time;



    while (true)
    {
        ret_value = 0;

        double cur_time = apollo::cyber::Time::Now().ToSecond();
        delta_time = (cur_time - history_time) * 1000.0;

        if (delta_time < 100.0)
        {
            continue;
        }

        AINFO << "print_plan_and_prediction_time:"
              << "(" << delta_time << ","
              << ")";

        const apollo::RunningTimeDebug *debug = apollo::get_debug_info();

        if (debug->pause_debug.enabled)
        {
            continue;
        }

        // planning_start_time = apollo::cyber::Time::Now().ToSecond();

        routing.update_routing_by_request(ptr_routing_response_, module_state,
                                          planning.is_new_routing_request(),
                                          planning.get_latest_routing_request());

        if (ptr_routing_response_->road_size() <= 0)
        {
            AERROR << "No routing";
            continue;
        }

        planning.update_cyber_frame_data();

        // 预测

        auto end_time1 = std::chrono::system_clock::now();

        const std::shared_ptr<apollo::perception::PerceptionObstacles>
                &perception = planning.get_const_perception();

        const std::shared_ptr<apollo::localization::LocalizationEstimate>
                &localization = planning.get_const_localization();

        std::shared_ptr<apollo::prediction::PredictionObstacles>
                prediction_result;
        prediction_result = planning.get_mutable_prediction();

        prediction_result->Clear();

        prediction.PredictionEndToEndProc(perception, localization,
                                          storytelling_msg,
                                          planning.get_latest_trajectory_ptr(),
                                          *(prediction_result.get()));

        auto end_time2 = std::chrono::system_clock::now();
        std::chrono::duration<double> pred_diff = end_time2 - end_time1;

        planning.process(ptr_routing_response_);

        // planning_end_time = apollo::cyber::Time::Now().ToSecond();

        // AINFO << "planning frame consume (ms): "
        //       << (planning_end_time - planning_start_time) * 1000;

        history_time = cur_time;
    }

    // terminate
    SystemStateFinish();
    apollo::cyber::Clear();

    google::ShutDownCommandLineFlags();

    return 0;
}
