

add_executable(test_shapes test_shapes.cpp)
target_link_libraries(test_shapes ${PCL_LIBRARIES})

add_executable(test_shapes_multiport test_shapes_multiport.cpp)
target_link_libraries(test_shapes_multiport ${PCL_LIBRARIES})

add_executable(text_text test_text.cpp)
target_link_libraries(text_text ${PCL_LIBRARIES})

add_executable(demo_text_simple_multiport text_simple_multiport.cpp)
target_link_libraries(demo_text_simple_multiport ${PCL_LIBRARIES})

add_executable(test_viz test_viz.cpp)
target_link_libraries(test_viz ${PCL_LIBRARIES}
gtest
gtest_main
)


add_executable(test_cyber_viz
        ${CMAKE_CURRENT_SOURCE_DIR}/cyber_viz.cc
        ./../../cyber/examples/proto/examples.pb.cc
        )
target_link_libraries(test_cyber_viz PRIVATE
        cyber
        ${PCL_LIBRARIES}
        
)
target_include_directories(test_cyber_viz PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        )



install(
    TARGETS 
    test_cyber_viz
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)
    