cmake_minimum_required(VERSION 3.5)


set(BUILD_TYPE "STATIC")


file(GLOB control_proto_file "proto/*.cc"
                                "proto/math/*.cc" 
                                )
add_library(control_proto  STATIC  ${control_proto_file})
target_include_directories(control_proto PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(control_proto PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         common_proto
                                         apollo_dreamview_proto
                                         )



file(GLOB control_file "common/*.cc"
                        "controller/*.cc" 
                        "submodules/*.cc" 
                        "tools/*.cc" 
)

add_library(apollo_control  ${BUILD_TYPE}  ${control_file})
target_include_directories(apollo_control PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(apollo_control PUBLIC
                                        apollo_planning
                                        apollo_common
                                         ${PROTOBUF_LIBRARIES}
                                         control_proto

                                         )


add_executable(control_component_main 
                ${CMAKE_CURRENT_SOURCE_DIR}/control_component_main.cc
                ${CMAKE_CURRENT_SOURCE_DIR}/control_component.cc
                )
target_link_libraries(control_component_main PUBLIC
                                                opencv_wrapper
                                                auto_virtual_chassis
                                                pthread

                                                fast_collision_detection
                                                apollo_planning
                                                apollo_control
                                                localization_proto
                                                localization_pose
                                                apollo_common
                                                
                                                )

install(TARGETS
        
        control_component_main
        apollo_control
        control_proto
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
        INCLUDES DESTINATION .
        )
