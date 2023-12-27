project(examples VERSION 1.0.0)

add_executable(cyber_example_talker talker.cc proto/examples.pb.cc)
target_link_libraries(cyber_example_talker
    cyber
    pthread
)

add_executable(cyber_example_listener listener.cc proto/examples.pb.cc)
target_link_libraries(cyber_example_listener
    cyber
    pthread
)

add_executable(cyber_example_paramserver paramserver.cc)
target_link_libraries(cyber_example_paramserver
    cyber
)

add_executable(cyber_example_record record.cc)
target_link_libraries(cyber_example_record
    cyber
)

add_executable(cyber_example_service service.cc proto/examples.pb.cc)
target_link_libraries(cyber_example_service
    cyber
)

add_executable(cyber_example_server server.cc proto/examples.pb.cc)
target_link_libraries(cyber_example_server
    cyber
)

add_executable(cyber_example_cilent client.cc proto/examples.pb.cc)
target_link_libraries(cyber_example_cilent
    cyber
)

add_subdirectory(common_component_example)

install(
        TARGETS 
        cyber_example_cilent
        cyber_example_server
        cyber_example_service
        cyber_example_record
        cyber_example_paramserver
        cyber_example_listener
        cyber_example_talker
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)

