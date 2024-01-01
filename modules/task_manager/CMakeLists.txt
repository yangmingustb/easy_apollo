cmake_minimum_required(VERSION 3.5)



#------------------------------
# complie apollo

set(BUILD_TYPE "STATIC")


file(GLOB task_manager_proto_file
    "proto/*.cc"



)

file(GLOB TASK_MANAGER_FILES 
    "common/*.cc"



)

add_library(task_manager_proto  ${BUILD_TYPE}  
        ${task_manager_proto_file}
        )
target_include_directories(task_manager_proto PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(task_manager_proto PUBLIC
                                         ${PROTOBUF_LIBRARIES}

                                         cyber
                                         apollo_common
                                         )

add_library(task_manager  ${BUILD_TYPE}  ${TASK_MANAGER_FILES})
target_include_directories(task_manager PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(task_manager PUBLIC
                                         ${PROTOBUF_LIBRARIES}

                                         cyber
                                         apollo_common
                                         task_manager_proto
                                         )
        

install(
        TARGETS 
        task_manager
        task_manager_proto
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)


