cmake_minimum_required(VERSION 3.5)


#----------------------------------

#apollo modules
file(GLOB_RECURSE APOLLO_MODULES_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.cc")




add_library(apollo_canbus  STATIC  ${APOLLO_MODULES_SOURCE_FILES})
target_include_directories(apollo_canbus PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(apollo_canbus PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         common_proto
                                         apollo_drivers
                                         )

install(TARGETS apollo_canbus
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        # RUNTIME DESTINATION bin
        INCLUDES DESTINATION .
        )

