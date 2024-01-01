cmake_minimum_required(VERSION 3.5)


set(BUILD_TYPE "STATIC")

find_package(PCL REQUIRED)
find_package(civetweb REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${civetweb_INCLUDE_DIR})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

# lib
file(GLOB DREAMVIEW_PLUS_PROTO_FILES 
    "proto/*.cc"


)

add_library(dreamview2_proto  SHARED  ${DREAMVIEW_PLUS_PROTO_FILES})
target_include_directories(dreamview2_proto PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(dreamview2_proto PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         apollo_common
                                         common_proto
                                         apollo_map
                                         apollo_monitor
                                         apollo_dreamview_proto
                                         common_msgs_proto
                                         )







# lib
add_library(dreamview2_hmi  ${BUILD_TYPE}  
backend/hmi/hmi_worker.cc
backend/hmi/hmi.cc



)
target_include_directories(dreamview2_hmi PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dreamview2_hmi PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					apollo_dreamview_proto
					dreamview2_proto
)


# lib
file(GLOB VIZ2_FILES 
    "backend/channels_updater/*.cc"
    "backend/dv_plugin/*.cc"
    "backend/map/*.cc"
    "backend/obstacle_updater/*.cc"

    "backend/perception_camera_updater/*.cc"
    "backend/point_cloud/*.cc"
    "backend/record_player/*.cc"

    "backend/simulation_world/*.cc"
    "backend/socket_manager/*.cc"
    "backend/updater/*.cc"


)

list(FILTER VIZ2_FILES EXCLUDE REGEX .*test[.]cc)

message("dreamview source: ${VIZ2_FILES}")

add_library(dreamview2_lib  ${BUILD_TYPE}
      ${VIZ2_FILES}
      backend/dreamview.cc
        )
target_include_directories(dreamview2_lib PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dreamview2_lib PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					apollo_dreamview_proto
					dreamview2_proto
                    
                    ${PCL_LIBRARIES}
                    ${civetweb_LIBRARIES}
                    libcivetweb-cpp.so
                    libcivetweb.so

                    cyber
                    gnss_driver
                    apollo_drivers
                    audio_proto
                    apollo_tf
                    yaml-cpp
                    task_manager

                    fuel_monitor
                    dreamview_common
                    dv_map
                    dv_handlers
                    dv_plugins
                    dv_sim_control_manager
                    task_manager
)

# exe
add_executable(dreamview2_main   main.cc)
target_include_directories(dreamview2_main PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(dreamview2_main PUBLIC
					${PROTOBUF_LIBRARIES}
					apollo_common
					apollo_map
					dreamview2_lib
					apollo_dreamview_proto

					dreamview2_proto
                    dreamview2_hmi

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
        
        dreamview2_proto
        dreamview2_lib
        dreamview2_main
        dreamview2_hmi
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
        INCLUDES DESTINATION .
        )
