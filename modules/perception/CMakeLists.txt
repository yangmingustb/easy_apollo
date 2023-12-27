cmake_minimum_required(VERSION 3.5)


#----------------------------------

#apollo modules
file(GLOB_RECURSE APOLLO_MODULES_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")


add_library(apollo_perception  SHARED  ${APOLLO_MODULES_SOURCE_FILES})
target_include_directories(apollo_perception PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(apollo_perception PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         apollo_map
                                         )


install(TARGETS apollo_perception

LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION .
)

