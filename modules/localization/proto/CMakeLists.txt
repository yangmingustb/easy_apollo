cmake_minimum_required(VERSION 3.5.1)



include_directories(${PROTOBUF_INCLUDE_DIRS})

add_library(localization_pose SHARED pose.pb.cc)
target_link_libraries(localization_pose
        ${PROTOBUF_LIBRARIES}
        common_geometry
        )


add_library(localization_proto SHARED 
                                        gnss_pnt_result.pb.cc
                                        gps.pb.cc
                                        imu.pb.cc
                                        localization_config.pb.cc
                                        localization_status.pb.cc
                                        localization.pb.cc
                                        measure.pb.cc
                                        rtk_config.pb.cc
                                        sins_pva.pb.cc
        )
target_link_libraries(localization_proto
        ${PROTOBUF_LIBRARIES}
        localization_pose
        gnss_driver
        common_header
        common_geometry
        common_pnc_point
        )

install(TARGETS
        localization_proto
        localization_pose
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION .
        )
