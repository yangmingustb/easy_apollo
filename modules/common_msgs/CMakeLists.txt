cmake_minimum_required(VERSION 3.5.1)



include_directories(${PROTOBUF_INCLUDE_DIRS})


file(GLOB MSGS_PROTO_FILES 
    "external_command_msgs/*.cc"
    "simulation_msgs/*.cc"


)

add_library(common_msgs_proto  STATIC  ${MSGS_PROTO_FILES})
target_include_directories(common_msgs_proto PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(common_msgs_proto PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         apollo_common
                                         common_proto
                                         apollo_map
                                         planning_proto
                                         )

install(TARGETS 
common_msgs_proto
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION .
        )
