cmake_minimum_required(VERSION 3.5)


#----------------------------------

add_subdirectory(proto)

file(GLOB COMMON_FILES 
    "adapters/proto/*.cc"
    "adapters/*.cc"

    "configs/*.cc"
    "configs/proto/*.cc"

    "filters/*.cc"
    "kv_db/*.cc"
    "math/*.cc"
    "math/qp_solver/*.cc"

    "monitor_log/proto/*.cc"
    "monitor_log/*.cc"

    "util/*.cc"

    "vehicle_model/*.cc"
    "vehicle_model/proto/*.cc"

    "vehicle_state/proto/*.cc"
    "vehicle_state/*.cc"
    
    "status/*.cc"
)



add_library(apollo_common  SHARED  ${COMMON_FILES})
target_include_directories(apollo_common PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_compile_options(apollo_common PRIVATE -Wno-deprecated-declarations)
target_link_libraries(apollo_common PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         cyber
                                        gflags
                                        common_proto
                                        localization_proto
                                        localization_common
                                         
                                        absl_strings
                                         absl_base
                                         absl_throw_delegate
                                         absl_str_format_internal

                                         osqp

                                         apollo_canbus
                                         sqlite3
                                         )

     
add_subdirectory(collision_detection)

install(TARGETS apollo_common
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION .
        )

