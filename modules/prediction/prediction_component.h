/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 */

#pragma once

#include <memory>
#include <string>

#include "cyber/time/time.h"

#include "cyber/component/component.h"
#include "modules/prediction/common/message_process.h"
#include "modules/prediction/container/adc_trajectory/adc_trajectory_container.h"
#include "modules/prediction/submodules/submodule_output.h"
#include "modules/storytelling/proto/story.pb.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo
{
namespace prediction
{
class PredictionComponent
{
public:
    /**
     * @brief Destructor
     */
    ~PredictionComponent();

    /**
     * @brief Get name of the node
     * @return Name of the node
     */
    std::string Name() const;

    /**
     * @brief Initialize the node
     * @return If initialized
     */
    bool Init();

    /**
     * @brief Data callback upon receiving a perception obstacle message.
     * @param Perception obstacle message.
     */
    // bool Proc(const std::shared_ptr<perception::PerceptionObstacles>&)
    // override;

    bool PredictionEndToEndProc(
            const std::shared_ptr<perception::PerceptionObstacles>
                    &perception_obstacles,
            const std::shared_ptr<localization::LocalizationEstimate>
                    &ptr_localization_msg,
            std::shared_ptr<storytelling::Stories> &ptr_storytelling_msg,
            const std::shared_ptr<planning::ADCTrajectory> &ptr_trajectory_msg,
            PredictionObstacles &prediction_obstacles);

    /**
     * @brief Load and process feature proto file.
     * @param a bin file including a sequence of feature proto.
     */
    void OfflineProcessFeatureProtoFile(const std::string &features_proto_file);

    void debug_prediction_info(PredictionObstacles *prediction_obstacles)
    {
        AINFO << "prediction obs size: "
              << prediction_obstacles->prediction_obstacle_size();

        for (const auto &prediction_obstacle :
             prediction_obstacles->prediction_obstacle())
        {
            const auto perception_id = std::to_string(
                    prediction_obstacle.perception_obstacle().id());

            AINFO << "perception id: " << perception_id;

            AINFO << " obs has multi traj size: "
                  << prediction_obstacle.trajectory_size();

            // int trajectory_index = 0;
            // for (const auto &trajectory : prediction_obstacle.trajectory())
            // {
            //     AINFO << "traj point size: "
            //           << trajectory.trajectory_point_size();

            //     const std::string obstacle_id =
            //             absl::StrCat(perception_id, "_", trajectory_index);

            //     AINFO << "prediction  id: " << obstacle_id;

            //     trajectory_index++;
            // }
        }

        return;
    }

private:
    // bool ContainerSubmoduleProcess(
    //    const std::shared_ptr<perception::PerceptionObstacles>&);

    double component_start_time_ = 0.0;
    double frame_start_time_ = 0.0;

#if 0
  std::shared_ptr<cyber::Reader<planning::ADCTrajectory>> planning_reader_;

  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      localization_reader_;

  std::shared_ptr<cyber::Reader<storytelling::Stories>> storytelling_reader_;

  std::shared_ptr<cyber::Writer<PredictionObstacles>> prediction_writer_;

  std::shared_ptr<cyber::Writer<SubmoduleOutput>> container_writer_;

  std::shared_ptr<cyber::Writer<ADCTrajectoryContainer>> adc_container_writer_;

  std::shared_ptr<cyber::Writer<perception::PerceptionObstacles>>
      perception_obstacles_writer_;
#endif

    std::shared_ptr<ContainerManager> container_manager_;

    std::unique_ptr<EvaluatorManager> evaluator_manager_;

    std::unique_ptr<PredictorManager> predictor_manager_;

    std::unique_ptr<ScenarioManager> scenario_manager_;

};

// CYBER_REGISTER_COMPONENT(PredictionComponent)

}  // namespace prediction
}  // namespace apollo
