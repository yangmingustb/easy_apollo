cmake_minimum_required(VERSION 3.5)



file(GLOB ROUTING_SRCS 
    "common/*.cc"
    "core/*.cc"
    "graph/*.cc"
    "strategy/*.cc"
    "proto/*.cc"
)


set(BUILD_TYPE "STATIC")

add_library(apollo_routing  ${BUILD_TYPE}  
                        ${ROUTING_SRCS}
                        routing.cc
                        routing_component.cc
                        )
target_include_directories(apollo_routing PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(apollo_routing PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         cyber
                                         apollo_common
                                         apollo_map
                                         )

