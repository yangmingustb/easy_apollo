cmake_minimum_required(VERSION 3.5)


# tf2
include_directories(${CMAKE_HOME_DIRECTORY}/third_party/install/tf2/include)
link_directories(${CMAKE_HOME_DIRECTORY}/third_party/install/tf2/lib)

set(BUILD_TYPE "STATIC")


file(GLOB APOLLO_TF_FILES 

    "proto/*.cc"
    "common/*.cc"

    "buffer.cc"
#     "tansform_broadcaster.cc"


)

add_library(apollo_tf  ${BUILD_TYPE}  ${APOLLO_TF_FILES})
target_include_directories(apollo_tf PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(apollo_tf PUBLIC
                                        #  ${DEP_LIBS}
                                         ${PROTOBUF_LIBRARIES}

                                         cyber
                                         apollo_common
                                         tf2
                                         )
        

install(
        TARGETS apollo_tf
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)


