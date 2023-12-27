cmake_minimum_required(VERSION 3.5)



#------------------------------
# complie apollo

set(BUILD_TYPE "STATIC")


file(GLOB TASK_MANAGER_FILES 
    "common/*.cc"
    "proto/*.cc"



)

add_library(task_manager  ${BUILD_TYPE}  ${TASK_MANAGER_FILES})
target_include_directories(task_manager PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(task_manager PUBLIC
                                        #  ${DEP_LIBS}
                                         ${PROTOBUF_LIBRARIES}

                                         cyber
                                         apollo_common
                                         )
        

install(
        TARGETS task_manager
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)


