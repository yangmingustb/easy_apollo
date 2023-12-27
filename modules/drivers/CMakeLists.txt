cmake_minimum_required(VERSION 3.5)



file(GLOB GNSS_FILES 
    "gnss/proto/*.cc"

)
add_library(gnss_driver  SHARED  ${GNSS_FILES})
target_include_directories(gnss_driver PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(gnss_driver PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         )



file(GLOB DRIVER_PROTO_FILES 
        "proto/*.cc"
    "camera/proto/*.cc"
    "canbus/proto/*.cc"
    "lidar/proto/*.cc"
    "microphone/proto/*.cc"
    "smartereye/proto/*.cc"
    "video/proto/*.cc"
    
    "radar/conti_radar/proto/*.cc"
    "radar/racobit_radar/proto/*.cc"
    "radar/ultrasonic_radar/proto/*.cc"
    


)

add_library(apollo_drivers  SHARED  ${DRIVER_PROTO_FILES})
target_include_directories(apollo_drivers PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(apollo_drivers PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                        #  ${OpenCV_LIBRARIES}
                                        #  cyber

                                         )


install(TARGETS gnss_driver
                apollo_drivers
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION .
        )
