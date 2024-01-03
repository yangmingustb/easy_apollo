cmake_minimum_required(VERSION 3.5)
set(BUILD_TYPE "STATIC")

file(GLOB prediction_proto_file "proto/*.cc"
                                )
add_library(prediction_proto  STATIC  ${prediction_proto_file})
target_include_directories(prediction_proto PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(prediction_proto PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         common_proto
                                         apollo_dreamview_proto
                                         )





file(GLOB prediction_file               "common/*.cc"
                                    "container/*.cc"
                                    "container/adc_trajectory/*.cc"
                                    "container/obstacles/*.cc"
                                    "container/pose/*.cc"
                                    "container/storytelling/*.cc"

                                    "evaluator/*.cc"
                                    "evaluator/cyclist/*.cc"
                                    "evaluator/pedestrian/*.cc"
                                    "evaluator/vehicle/*.cc"


                                    "network/*.cc"
                                    "network/rnn_model/*.cc"

                                    "pipeline/*.cc"

                                    "predictor/*.cc"
                                    "predictor/empty/*.cc"
                                    "predictor/extrapolation/*.cc"
                                    "predictor/free_move/*.cc"
                                    "predictor/interaction/*.cc"
                                    "predictor/junction/*.cc"
                                    "predictor/lane_sequence/*.cc"
                                    "predictor/move_sequence/*.cc"
                                    "predictor/sequence/*.cc"
                                    "predictor/single_lane/*.cc"

                                    "scenario/*.cc"
                                    "scenario/analyzer/*.cc"
                                    "scenario/feature_extractor/*.cc"
                                    "scenario/interaction_filter/*.cc"
                                    "scenario/prioritization/*.cc"
                                    "scenario/right_of_way/*.cc"
                                    "scenario/scenario_features/*.cc"


                                    "submodules/*.cc"

                                    "util/*.cc"
        )

add_library(apollo_prediction  ${BUILD_TYPE}  ${prediction_file}
prediction_component.cc
)
target_include_directories(apollo_prediction PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )

target_compile_options(apollo_prediction PRIVATE -Wno-deprecated-declarations)
target_link_libraries(apollo_prediction PRIVATE
                                         ${PROTOBUF_LIBRARIES}
                                         ${OpenCV_LIBRARIES}
                                         ${PYTHON_LIBRARIES}
                                         ${TORCH_LIBRARIES}

                                         cyber
                                         apollo_common
                                         apollo_map
                                         apollo_routing
                                         prediction_proto

                                         )



install(TARGETS
prediction_proto
        apollo_prediction

                                         
                                         LIBRARY DESTINATION lib
                                         ARCHIVE DESTINATION lib
                                         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
                                         INCLUDES DESTINATION .
                                         )