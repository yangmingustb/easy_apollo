cmake_minimum_required(VERSION 3.5)


#----------------------------------

#apollo modules
file(GLOB_RECURSE APOLLO_MODULES_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")


#换行打印
# message(STATUS "apollo modules： ")
# foreach(_var ${APOLLO_MODULES_SOURCE_FILES})
#    message("   ${_var}")
# endforeach()


#------------------------------
# complie apollo

set(BUILD_TYPE "STATIC")

add_library(apollo_prediction  ${BUILD_TYPE}  ${APOLLO_MODULES_SOURCE_FILES})
target_include_directories(apollo_prediction PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )

target_compile_options(apollo_prediction PRIVATE -Wno-deprecated-declarations)
target_link_libraries(apollo_prediction PUBLIC
                                         ${PROTOBUF_LIBRARIES}
                                         ${OpenCV_LIBRARIES}
                                         ${PYTHON_LIBRARIES}
                                         ${TORCH_LIBRARIES}

                                         cyber
                                         apollo_common
                                         apollo_map
                                         apollo_routing

                                         )



install(TARGETS
        
        apollo_prediction
                                         
                                         LIBRARY DESTINATION lib
                                         ARCHIVE DESTINATION lib
                                         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
                                         INCLUDES DESTINATION .
                                         )