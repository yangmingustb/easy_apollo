cmake_minimum_required(VERSION 3.5)

# set(BUILD_TYPE "SHARED")
set(BUILD_TYPE "STATIC")

add_compile_options(-fPIC)

find_package(Protobuf REQUIRED)
find_package(PCL)

add_library(viz3d ${BUILD_TYPE} 
                    viz3d_component.cc
                    draw_path.cc
                    draw_geometry.cc
                    draw_map.cc
                    pcl_viz.cc
)

target_include_directories(viz3d PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    )
target_link_libraries(viz3d PUBLIC 
        #     apollo_dreamview_lib
        #     fast_collision_detection
        #     apollo_planning
        #     apollo_control
        #     apollo_map
        #     localization_common
        #     apollo_perception
        #     apollo_routing
            localization_proto
        #     localization_pose
        #     apollo_prediction
        #     apollo_common
        #     cyber
        
            viz2d_proto
            apollo_map
            routing_proto
            prediction_proto
            control_proto
            planning_proto
            opencv_wrapper
            dv_map
            ${PCL_LIBRARIES}
            planning_gflags
            
            )



add_executable(viz3d_component_main 
                ${CMAKE_CURRENT_SOURCE_DIR}/main.cc
                )
target_link_libraries(viz3d_component_main PUBLIC
viz3d
cyber
)


install(
        TARGETS 
        viz3d
        viz3d_component_main
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)
