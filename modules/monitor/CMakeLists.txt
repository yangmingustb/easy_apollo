cmake_minimum_required(VERSION 3.5)


#----------------------------------

#apollo modules
file(GLOB_RECURSE APOLLO_MODULES_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")



add_library(apollo_monitor  SHARED  ${APOLLO_MODULES_SOURCE_FILES})
target_include_directories(apollo_monitor PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(apollo_monitor PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         )

install(TARGETS apollo_monitor

LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION .
)

