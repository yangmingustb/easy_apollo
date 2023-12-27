cmake_minimum_required(VERSION 3.5)

set(BUILD_TYPE "STATIC")
# set(BUILD_TYPE "SHARED")
add_compile_options(-fPIC)


add_library(auto_virtual_chassis ${BUILD_TYPE}
                     virtual_chassis.cc
                     virtual_chassis_component.cc
                    )
target_include_directories(auto_virtual_chassis PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(auto_virtual_chassis PRIVATE
                opencv_wrapper
                )

add_library(auto_chassis ${BUILD_TYPE}
                    chassis_component.cc
                    )
target_include_directories(auto_chassis PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(auto_chassis       PRIVATE
                opencv_wrapper
                )

add_executable(chassis_test
            ${CMAKE_CURRENT_SOURCE_DIR}/virtual_chassis_test.cc
)
target_link_libraries(chassis_test 
                auto_virtual_chassis

)

add_executable(virtual_chassis_component_main 
                ${CMAKE_CURRENT_SOURCE_DIR}/virtual_chassis_component_main.cc
                )
target_link_libraries(virtual_chassis_component_main PUBLIC
                                                opencv_wrapper
                                                auto_virtual_chassis
                                                # ${TORCH_LIBRARIES}
                                                pthread
                                                )


install(
        TARGETS virtual_chassis_component_main
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)