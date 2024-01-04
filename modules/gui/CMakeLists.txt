cmake_minimum_required(VERSION 3.5)

# set(BUILD_TYPE "SHARED")
set(BUILD_TYPE "STATIC")

add_compile_options(-fPIC)


add_library(imgui ${BUILD_TYPE} 
imgui/imgui.cpp 
imgui/imgui_draw.cpp 
imgui/imgui_tables.cpp 
imgui/imgui_widgets.cpp 
imgui/imgui_demo.cpp 
imgui/backends/imgui_impl_glfw.cpp # platform
imgui/backends/imgui_impl_opengl3.cpp # render

)

target_include_directories(imgui PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
    )
target_link_libraries(imgui PUBLIC 
glfw
GL
glut
${CMAKE_DL_LIBS}

            )



add_executable(test_glfw 
                test/test_glfw.cc
                )
target_link_libraries(test_glfw PUBLIC
imgui
)

add_executable(test_gl
                test/test_gl.cc
                )
target_link_libraries(test_gl PUBLIC
imgui
)


install(
        TARGETS 
        test_glfw
        imgui
        test_gl
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)
