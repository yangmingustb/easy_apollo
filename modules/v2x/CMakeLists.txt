cmake_minimum_required(VERSION 3.5)




set(BUILD_TYPE "STATIC")


file(GLOB V2X_FILES 
    "proto/*.cc"


)

add_library(apollo_v2x  ${BUILD_TYPE}  ${V2X_FILES})
target_include_directories(apollo_v2x PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
target_link_libraries(apollo_v2x PUBLIC
                                        #  ${DEP_LIBS}
                                         ${PROTOBUF_LIBRARIES}

                                         cyber
                                         apollo_common
                                         )
        

install(
        TARGETS apollo_v2x
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)


