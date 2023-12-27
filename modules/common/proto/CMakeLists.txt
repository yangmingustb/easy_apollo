cmake_minimum_required(VERSION 3.5.1)



include_directories(${PROTOBUF_INCLUDE_DIRS})

add_library(error_code SHARED 
                            error_code.pb.cc
)
target_link_libraries(error_code ${PROTOBUF_LIBRARIES})

add_library(common_header SHARED 
                                header.pb.cc
)
target_link_libraries(common_header ${PROTOBUF_LIBRARIES}
                                    error_code)

add_library(drive_event SHARED 
                                drive_event.pb.cc
)
target_link_libraries(drive_event ${PROTOBUF_LIBRARIES}
                                    common_header
                                    localization_pose)

add_library(common_geometry SHARED 
    geometry.pb.cc
)
target_link_libraries(common_geometry ${PROTOBUF_LIBRARIES}
                                    )

add_library(common_pnc_point SHARED 
            pnc_point.pb.cc
)
target_link_libraries(common_pnc_point ${PROTOBUF_LIBRARIES}
                                    )

add_library(common_proto SHARED 
                                direction.pb.cc
                                drive_state.pb.cc
                                vehicle_signal.pb.cc
)
target_link_libraries(common_proto ${PROTOBUF_LIBRARIES}
                                    common_header
                                    drive_event
                                    common_geometry
)

install(TARGETS 
        common_proto
        error_code
        common_header
        drive_event
        common_geometry
        common_pnc_point
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION .
        )
