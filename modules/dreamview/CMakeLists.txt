cmake_minimum_required(VERSION 3.5)


set(BUILD_TYPE "STATIC")

find_package(PCL REQUIRED)
find_package(civetweb REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${civetweb_INCLUDE_DIR})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

# lib
file(GLOB VIZ_PROTO_FILES 
    "backend/teleop/proto/*.cc"
    "proto/*.cc"


)

add_library(apollo_dreamview_proto  SHARED  ${VIZ_PROTO_FILES})
target_include_directories(apollo_dreamview_proto PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(apollo_dreamview_proto PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         apollo_common
                                         common_proto
                                         apollo_map
                                         apollo_monitor
                                         common_msgs_proto
                                         )






# lib
add_library(fuel_monitor  ${BUILD_TYPE}  
backend/fuel_monitor/data_collection_monitor.cc
backend/fuel_monitor/fuel_monitor_gflags.cc
backend/fuel_monitor/fuel_monitor_manager.cc
backend/fuel_monitor/preprocess_monitor.cc



)
target_include_directories(fuel_monitor PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(fuel_monitor PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					apollo_dreamview_proto
)

# lib
add_library(apollo_hmi  ${BUILD_TYPE}  
backend/hmi/hmi_worker.cc
backend/hmi/hmi.cc
backend/hmi/vehicle_manager.cc



)
target_include_directories(apollo_hmi PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(apollo_hmi PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					apollo_dreamview_proto
                    fuel_monitor
)


# lib
file(GLOB VIZ_FILES 
    "backend/common/*.cc"
    "backend/common/util/*.cc"
    # "backend/fuel_monitor/*.cc"
    "backend/handlers/*.cc"
    # "backend/hmi/*.cc"
    "backend/map/*.cc"
    "backend/perception_camera_updater/*.cc"
    "backend/point_cloud/*.cc"
    "backend/sim_control/*.cc"
    "backend/simulation_world/*.cc"

    "backend/sim_control_manager/*.cc"
    "backend/sim_control_manager/proto/*.cc"
    "backend/sim_control_manager/common/*.cc"
    "backend/sim_control_manager/core/*.cc"
    "backend/sim_control_manager/dynamic_model/perfect_control/*.cc"
    "backend/sim_control_manager/dynamic_model/perfect_control/*.cc"


)

list(FILTER VIZ_FILES EXCLUDE REGEX .*test[.]cc)

message("dreamview source: ${VIZ_FILES}")

add_library(apollo_dreamview_lib  ${BUILD_TYPE}
      ${VIZ_FILES}
      backend/dreamview.cc
        )
target_include_directories(apollo_dreamview_lib PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(apollo_dreamview_lib PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					apollo_dreamview_proto
                    apollo_hmi
                    fuel_monitor
                    ${PCL_LIBRARIES}
                    ${civetweb_LIBRARIES}
                    libcivetweb-cpp.so
                    libcivetweb.so

                    gnss_driver
                    apollo_drivers
                    audio_proto
                    apollo_tf
                    yaml-cpp
                    task_manager
)

# exe
add_executable(dreamview_main   main.cc)
target_include_directories(dreamview_main PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dreamview_main PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					apollo_dreamview_lib
					apollo_dreamview_proto
                    apollo_monitor
                    apollo_planning
                    apollo_control
                    localization_common
                    localization_proto
                    apollo_routing
                    apollo_prediction
                    apollo_canbus
                    apollo_storytelling
                    localization_pose
                    apollo_perception
)



install(TARGETS
        
        apollo_dreamview_proto
        apollo_dreamview_lib
        dreamview_main
        apollo_hmi
        fuel_monitor
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
        INCLUDES DESTINATION .
        )
