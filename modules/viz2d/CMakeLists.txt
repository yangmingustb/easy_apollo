cmake_minimum_required(VERSION 3.5)

# set(BUILD_TYPE "SHARED")
set(BUILD_TYPE "STATIC")

add_compile_options(-fPIC)

find_package(Protobuf REQUIRED)

SET(PROTO_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})
LIST(APPEND PROTO_FLAGS -I${CMAKE_CURRENT_SOURCE_DIR})


# Generate the .h and .cxx files
file(GLOB viz2d_proto_file ${CMAKE_CURRENT_SOURCE_DIR}/*.proto
)

PROTOBUF_GENERATE_CPP(MSGS_PROTO_SRCS MSGS_PROTO_HDRS ${viz2d_proto_file})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

include_directories(include
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    )

add_library(viz2d_proto STATIC ${MSGS_PROTO_SRCS} ${MSGS_PROTO_HDRS})
target_include_directories(viz2d_proto PUBLIC
                ${CMAKE_CURRENT_BINARY_DIR}
                ${CMAKE_CURRENT_SOURCE_DIR}
                ${Protobuf_INCLUDE_DIRS}
                $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX})
target_link_libraries(viz2d_proto PUBLIC ${PROTOBUF_LIBRARIES})


#  opencv, cyber相关的wrapper
add_library(opencv_wrapper ${BUILD_TYPE} 
                    opencv_viz.cc
                    viz_window.cc
                    viz2d_component.cc
                    viz2d_map_elements.cc
                    viz2d_traffic_light.cc
                    viz2d_perception.cc
                    viz2d_obstacle_decision.cc
                    viz2d_path.cc
                    viz2d_routing.cc
                    viz2d_prediction.cc
                    viz2d_geometry.cc
                    viz2d_lane_decision.cc
                    viz2d_key.cc
                    viz2d_state.cc
)

target_include_directories(opencv_wrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_HOME_DIRECTORY}/build
    )
target_link_libraries(opencv_wrapper PUBLIC 
            ${OpenCV_LIBRARIES}
            apollo_dreamview_lib
            fast_collision_detection
            planning_proto
            control_proto
            apollo_map
            localization_common
            apollo_perception
            routing_proto
            localization_proto
            localization_pose
            prediction_proto
            apollo_common
            cyber
            viz2d_proto
            
            )


add_executable(test_opencv2d
        ${CMAKE_CURRENT_SOURCE_DIR}/viz_test.cc
        )
target_link_libraries(test_opencv2d PUBLIC
        gtest
        gtest_main
        ${OpenCV_LIBRARIES}
        opencv_wrapper
        
)




add_executable(viz2d_component_main 
                ${CMAKE_CURRENT_SOURCE_DIR}/viz2d_component_main.cc
                )
target_link_libraries(viz2d_component_main PUBLIC
                                                opencv_wrapper
                                                pthread
                                                viz2d_proto
                                                apollo_storytelling
                                                apollo_prediction
                                                apollo_routing
                                                apollo_map
                                                apollo_common
                                                apollo_planning
                                                apollo_control
                                                apollo_perception
                                                )


install(
        TARGETS 
            test_opencv2d
            viz2d_component_main
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)
