cmake_minimum_required(VERSION 3.5)



#------------------------------
# complie apollo

set(BUILD_TYPE "STATIC")


file(GLOB MAP_FILES 
    "hdmap/*.cc"
    "hdmap/adapter/*.cc"
    "hdmap/adapter/xml_parser/*.cc"
    
    
    "pnc_map/*.cc"

    "proto/*.cc"

    "relative_map/proto/*.cc"

    "tools/map_datachecker/proto/*.cc"

)

add_library(apollo_map  ${BUILD_TYPE}  ${MAP_FILES})
target_include_directories(apollo_map PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(apollo_map PUBLIC
                                         ${PROTOBUF_LIBRARIES}

                                         cyber
                                         apollo_common
                                         proj
                                         tinyxml2
                                         
                                         absl_strings
                                         absl_base
                                         absl_throw_delegate
                                         absl_str_format_internal
                                         )
        

add_executable(apollo_txt_map_tool   tools/txt_map_generator.cc)
target_include_directories(apollo_txt_map_tool PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(apollo_txt_map_tool PUBLIC
                                        ${PROTOBUF_LIBRARIES}
                                        cyber
                                        apollo_common
                                        apollo_map
                                        )

add_executable(generate_topo_txt   tools/generate_topo_txt.cc)
target_include_directories(generate_topo_txt PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(generate_topo_txt PUBLIC
                                        ${PROTOBUF_LIBRARIES}
                                        cyber
                                        apollo_common
                                        apollo_map
                                        apollo_routing
                                        )
install(
        TARGETS 
                apollo_txt_map_tool
                generate_topo_txt
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)


