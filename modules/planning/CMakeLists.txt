cmake_minimum_required(VERSION 3.5)

set(BUILD_TYPE "STATIC")



add_library(planning_gflags  ${BUILD_TYPE}                       
common/planning_gflags.cc
                                        )
target_include_directories(planning_gflags PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(planning_gflags PUBLIC
gflags
                                        )

file(GLOB apollo_planning_common 
                                "common/decision_data.cc"
                                "common/ego_info.cc"
                                "common/frame.cc"
                                "common/feature_output.cc"
                                "common/history.cc"
                                "common/learning_based_data.cc"
                                "common/message_process.cc"
                                "common/obstacle_blocking_analyzer.cc"
                                "common/obstacle.cc"
                                "common/open_space_info.cc"
                                "common/path_boundary.cc"
                                "common/path_decision.cc"
                                "common/planning_context.cc"
                                "common/reference_line_info.cc"
                                "common/sl_boundary.cc"
                                "common/speed_limit.cc"
                                "common/speed_profile_generator.cc"
                                "common/st_graph_data.cc"
                                "common/trajectory_evaluator.cc"
                                "common/trajectory_stitcher.cc"

                                "common/path/*.cc"
                                "common/smoothers/*.cc"
                                "common/speed/*.cc"
                                "common/trajectory/*.cc"
                                "common/trajectory1d/*.cc"
                                "common/util/*.cc"
                                )
file(GLOB apollo_planning_constraint "constraint_checker/*.cc" )

file(GLOB apollo_planning_lattice "lattice/behavior/*.cc"
                                    "lattice/trajectory_generation/*.cc"    
                                    )

file(GLOB apollo_planning_learning "learning_based/img_feature_renderer/*.cc"
                                    "learning_based/model_inference/*.cc"    
                                    )

file(GLOB apollo_planning_math "math/*.cc"
                                    "math/curve1d/*.cc"    
                                    "math/discretized_points_smoothing/*.cc"    
                                    "math/piecewise_jerk/*.cc"    
                                    "math/smoothing_spline/*.cc"    
                                    )

file(GLOB apollo_planning_navi "navi/decider/*.cc" )

file(GLOB apollo_planning_openspace "open_space/coarse_trajectory_generator/*.cc"
                                    "open_space/tools/*.cc" 
                                    "open_space/trajectory_smoother/*.cc" 
                                    )

file(GLOB apollo_planning_pipeline "pipeline/*.cc" )

file(GLOB apollo_planning_planner "planner/*.cc"
                                    "planner/lattice/*.cc"
                                    "planner/navi/*.cc"
                                    "planner/public_road/*.cc"
                                    "planner/rtk/*.cc"
 )

file(GLOB apollo_planning_proto "proto/*.cc"
                                "proto/math/*.cc" )

file(GLOB apollo_planning_ref_line "reference_line/*.cc" )

file(GLOB apollo_planning_scenario "scenarios/*.cc"
                                    "scenarios/bare_intersection/unprotected/*.cc"
                                    "scenarios/common/*.cc"
                                    "scenarios/dead_end/deadend_turnaround/*.cc"
                                    "scenarios/emergency/emergency_pull_over/*.cc"
                                    "scenarios/emergency/emergency_stop/*.cc"
                                    "scenarios/lane_follow/*.cc"
                                    "scenarios/learning_model/*.cc"
                                    "scenarios/narrow_street_u_turn/*.cc"
                                    "scenarios/park/pull_over/*.cc"
                                    "scenarios/park/valet_parking/*.cc"
                                    "scenarios/park_and_go/*.cc"
                                    "scenarios/stop_sign/unprotected/*.cc"
                                    "scenarios/traffic_light/protected/*.cc"
                                    "scenarios/traffic_light/unprotected_left_turn/*.cc"
                                    "scenarios/traffic_light/unprotected_right_turn/*.cc"
                                    "scenarios/util/*.cc"
                                    "scenarios/yield_sign/*.cc"
        )
        
file(GLOB apollo_planning_task "tasks/*.cc"
                                "tasks/utils/*.cc"

                                "tasks/optimizers/*.cc"
                                
                                "tasks/optimizers/open_space_trajectory_generation/*.cc"
                                "tasks/optimizers/open_space_trajectory_partition/*.cc"


                                "tasks/optimizers/path_time_heuristic/*.cc"
                                "tasks/optimizers/piecewise_jerk_path/*.cc"
                                "tasks/optimizers/piecewise_jerk_speed/*.cc"
                                "tasks/optimizers/road_graph/*.cc"

                                "tasks/deciders/*.cc"
                                "tasks/deciders/creep_decider/*.cc"
                                "tasks/deciders/lane_change_decider/*.cc"

                                "tasks/deciders/open_space_decider/*.cc"

                                "tasks/deciders/path_assessment_decider/*.cc"
                                "tasks/deciders/path_bounds_decider/*.cc"
                                "tasks/deciders/path_decider/*.cc"
                                "tasks/deciders/path_lane_borrow_decider/*.cc"
                                "tasks/deciders/path_reference_decider/*.cc"
                                "tasks/deciders/path_reuse_decider/*.cc"

                                "tasks/deciders/rss_decider/*.cc"
                                
                                "tasks/deciders/rule_based_stop_decider/*.cc"
                                "tasks/deciders/speed_bounds_decider/*.cc"
                                "tasks/deciders/speed_decider/*.cc"
                                "tasks/deciders/st_bounds_decider/*.cc"
                                "tasks/deciders/utils/*.cc"

                                "tasks/learning_model/*.cc"

)

file(GLOB apollo_planning_traffic_rule "traffic_rules/*.cc" )

file(GLOB apollo_planning_tuning "tuning/*.cc"
                                    "tuning/speed_model/*.cc" )









add_library(planning_proto  STATIC  ${apollo_planning_proto})
target_include_directories(planning_proto PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(planning_proto PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         apollo_common
                                         common_proto
                                         apollo_dreamview_proto
                                         apollo_perception
                                         routing_proto
                                         apollo_map
                                         apollo_canbus
                                         )

add_library(apollo_planning  ${BUILD_TYPE}                       
                                        navi_planning.cc
                                        on_lane_planning.cc
                                        planning_base.cc

                                        ${apollo_planning_common}
                                        ${apollo_planning_constraint}
                                        ${apollo_planning_lattice}
                                        ${apollo_planning_math}
                                        ${apollo_planning_navi}
                                        ${apollo_planning_pipeline}
                                        ${apollo_planning_planner}
                                        ${apollo_planning_ref_line}
                                        ${apollo_planning_scenario}
                                        ${apollo_planning_task}
                                        ${apollo_planning_traffic_rule}

                                        ${apollo_planning_tuning}
                                        ${apollo_planning_openspace}
                                        ${apollo_planning_learning}
                                        )
target_include_directories(apollo_planning PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        )
target_link_libraries(apollo_planning PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         cyber
                                         apollo_common
                                         apollo_map
                                         apollo_routing
                                         apollo_dreamview_proto
                                         planning_proto

                                         ${OpenCV_LIBRARIES}

                                        ipopt
                                        # boost_system
                                        boost_filesystem
                                        boost_thread

                                        adolc
                                        
                                        ad-rss
                                        osqp
                                        # ${PYTHON_LIBRARIES}
                                        fast_collision_detection
                                        planning_gflags
                                        
                                        )

add_executable(planning_component_main 
                                        planning_component_main.cc
                                        planning_component.cc
                                        )
target_link_libraries(planning_component_main PUBLIC
                                        opencv_wrapper
                                        ${TORCH_LIBRARIES}
                                        pthread
                                        apollo_storytelling
                                        apollo_prediction
                                        apollo_routing
                                        apollo_map
                                        apollo_common
                                        apollo_perception
                                        apollo_planning
)


install(
TARGETS 
apollo_planning
        planning_proto
        planning_component_main
        planning_gflags
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)