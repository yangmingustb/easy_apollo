cmake_minimum_required(VERSION 3.5)


add_compile_options(-fPIC)


find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

#test routing
add_executable(test_routing 
                    ${CMAKE_CURRENT_SOURCE_DIR}/test_routing.cc)
target_link_libraries(test_routing PUBLIC
        gtest
        gtest_main
        apollo_routing
)


add_executable(test_talker talker.cc
                        ./../cyber/examples/proto/examples.pb.cc)
target_link_libraries(test_talker cyber
# apollo
# ${torch_cpu_LIBRARIES}
)

add_executable(test_listener listener.cc
                                ./../cyber/examples/proto/examples.pb.cc)
target_link_libraries(test_listener cyber)


add_executable(test_talker_null test_publish_null_msg.cc
                        ./../cyber/examples/proto/examples.pb.cc)
target_link_libraries(test_talker_null cyber
# apollo
# ${torch_cpu_LIBRARIES}
)

add_executable(test_listener_null test_subscribe_null_msg.cc
                                ./../cyber/examples/proto/examples.pb.cc)
target_link_libraries(test_listener_null cyber)

add_executable(test_visualization
        ${CMAKE_CURRENT_SOURCE_DIR}/test_pcl_viz/test_visualization.cc
        )
target_link_libraries(test_visualization PRIVATE
        gtest
        gtest_main
        ${PCL_LIBRARIES}
        
)
target_include_directories(test_visualization PUBLIC
${PCL_INCLUDE_DIRS}
)

add_subdirectory(test_pcl_viz)

install(
        TARGETS 
                test_routing
                test_talker
                test_listener
                test_talker_null
                test_listener_null
                test_visualization
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)

