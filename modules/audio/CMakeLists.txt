cmake_minimum_required(VERSION 3.5)





file(GLOB AUDIO_PROTO_FILES 
    "proto/*.cc"


)

add_library(audio_proto  STATIC  ${AUDIO_PROTO_FILES})
target_include_directories(audio_proto PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(audio_proto PUBLIC
                                         ${PROTOBUF_LIBRARIES}

                                         )


install(TARGETS 
        audio_proto
                
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        # RUNTIME DESTINATION bin
        INCLUDES DESTINATION .
        )
