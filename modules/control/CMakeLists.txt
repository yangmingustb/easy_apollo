cmake_minimum_required(VERSION 3.5)


#----------------------------------

#apollo modules
file(GLOB_RECURSE APOLLO_MODULES_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")


#换行打印
# message(STATUS "apollo modules： ")
# foreach(_var ${APOLLO_MODULES_SOURCE_FILES})
#    message("   ${_var}")
# endforeach()


#------------------------------
# complie apollo

set(BUILD_TYPE "STATIC")

add_library(apollo_control  ${BUILD_TYPE}  ${APOLLO_MODULES_SOURCE_FILES})
target_include_directories(apollo_control PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(apollo_control PUBLIC
                                        apollo_planning
                                        apollo_common
                                         ${PROTOBUF_LIBRARIES}
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
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
        INCLUDES DESTINATION .
        )
