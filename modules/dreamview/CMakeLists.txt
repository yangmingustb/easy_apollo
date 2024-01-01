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

add_library(apollo_dreamview_proto  STATIC  ${VIZ_PROTO_FILES})
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
add_library(dv_map  ${BUILD_TYPE}  
backend/map/map_service.cc



)
target_include_directories(dv_map PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dv_map PUBLIC
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
                    apollo_control
)


# lib
file(GLOB dreamview_common_file
    "backend/common/*.cc"
    "backend/common/util/*.cc"
)

list(FILTER dreamview_common_file EXCLUDE REGEX .*test[.]cc)

message("dreamview source: ${dreamview_common_file}")

add_library(dreamview_common  ${BUILD_TYPE}
      ${dreamview_common_file}
        )
target_include_directories(dreamview_common PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dreamview_common PUBLIC
					apollo_common
					apollo_dreamview_proto
)

# lib
add_library(dv_handlers  ${BUILD_TYPE}  
backend/handlers/image_handler.cc
backend/handlers/proto_handler.cc
backend/handlers/websocket_handler.cc



)
target_include_directories(dv_handlers PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dv_handlers PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					apollo_dreamview_proto
                    ${civetweb_LIBRARIES}
)


# lib
add_library(dv_pcu  ${BUILD_TYPE}  
backend/perception_camera_updater/perception_camera_updater.cc



)
target_include_directories(dv_pcu PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dv_pcu PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					apollo_dreamview_proto
)

# lib
add_library(dv_plugins  ${BUILD_TYPE}  
backend/plugins/plugin_manager.cc



)
target_include_directories(dv_plugins PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dv_plugins PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					apollo_dreamview_proto
)

# lib
add_library(dv_simulation_world  ${BUILD_TYPE}  
backend/simulation_world/simulation_world_service.cc
backend/simulation_world/simulation_world_updater.cc



)
target_include_directories(dv_simulation_world PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dv_simulation_world PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					apollo_dreamview_proto
                    apollo_storytelling
)

# lib
add_library(dv_point_cloud  ${BUILD_TYPE}  
backend/point_cloud/point_cloud_updater.cc



)
target_include_directories(dv_point_cloud PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dv_point_cloud PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					apollo_dreamview_proto
                    yaml-cpp
)


# lib
file(GLOB dv_sim_control_manager_file 

    "backend/sim_control_manager/*.cc"
    "backend/sim_control_manager/proto/*.cc"
    "backend/sim_control_manager/common/*.cc"
    "backend/sim_control_manager/core/*.cc"
    "backend/sim_control_manager/dynamic_model/perfect_control/*.cc"
    "backend/sim_control_manager/dynamic_model/perfect_control/*.cc"


)

list(FILTER dv_sim_control_manager_file EXCLUDE REGEX .*test[.]cc)

message("dreamview source: ${dv_sim_control_manager_file}")

add_library(dv_sim_control_manager  ${BUILD_TYPE}
      ${dv_sim_control_manager_file}
        )
target_include_directories(dv_sim_control_manager PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dv_sim_control_manager PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					apollo_dreamview_proto
)

# lib
file(GLOB VIZ_FILES 
    # "backend/fuel_monitor/*.cc"
    # "backend/handlers/*.cc"
    # "backend/hmi/*.cc"
    # "backend/perception_camera_updater/*.cc"
    # "backend/point_cloud/*.cc"
    "backend/sim_control/*.cc"
    # "backend/simulation_world/*.cc"



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
                    dreamview_common
                    fuel_monitor
                    dv_map
                    dv_handlers
                    dv_pcu
                    dv_simulation_world
                    dv_point_cloud
                    dv_sim_control_manager
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
                    apollo_hmi
)



install(TARGETS
        
        apollo_dreamview_proto
        apollo_dreamview_lib
        dreamview_main
        apollo_hmi
        fuel_monitor
        dreamview_common
        dv_map
        dv_handlers
        dv_pcu
        dv_plugins
        dv_simulation_world
        dv_point_cloud
        dv_sim_control_manager

        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
        INCLUDES DESTINATION .
        )
