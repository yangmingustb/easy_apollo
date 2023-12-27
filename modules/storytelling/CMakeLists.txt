cmake_minimum_required(VERSION 3.5)

set(BUILD_TYPE "STATIC")


file(GLOB APOLLO_STORY_FILES 
    "proto/*.cc"


)

add_library(apollo_storytelling  ${BUILD_TYPE}  ${APOLLO_STORY_FILES})
target_include_directories(apollo_storytelling PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(apollo_storytelling PUBLIC
                                         ${PROTOBUF_LIBRARIES}

                                         cyber
                                         apollo_common
                                         )

