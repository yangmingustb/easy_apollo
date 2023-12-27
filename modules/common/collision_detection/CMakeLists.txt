# cmake_minimum_required(VERSION 3.5.1)

# set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
# include(GNUInstallDirs)

if(BUILD_SHARED_LIBRARY)
  set(BUILD_TYPE "SHARED")
else(BUILD_SHARED_LIBRARY)
  add_definitions(-DBUILD_STATIC_LIBRARY)
  set(BUILD_TYPE "STATIC")
endif(BUILD_SHARED_LIBRARY)

## enable optimazition
add_compile_options(-O3)

file(GLOB collision_detection_file 
    "*.cc"
)

add_library(fast_collision_detection ${BUILD_TYPE}

                                ${collision_detection_file}
                                  )
target_compile_options(fast_collision_detection PRIVATE -fPIC -Wall -Wextra -Wshadow -Wno-missing-field-initializers -Werror  -Wunreachable-code -Wunused -Wno-inconsistent-missing-override -Wno-ignored-qualifiers -Wno-deprecated-copy -Wno-unused-but-set-variable)
# target_include_directories(collision_detection PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(fast_collision_detection
                              apollo_common
                              )

#Installation
install(
  TARGETS
       fast_collision_detection
   LIBRARY DESTINATION lib
   ARCHIVE DESTINATION lib
   RUNTIME DESTINATION bin
   INCLUDES DESTINATION .
)


add_executable(gjk_main_test test/gjk_main_test.cc)
target_link_libraries(gjk_main_test gtest gtest_main fast_collision_detection)

add_executable(gjk_interface_test test/gjk_interface_test.cc)
target_link_libraries(gjk_interface_test gtest gtest_main fast_collision_detection)

add_executable(polygon_test test/polygon_test.cc)
target_link_libraries(polygon_test 
    gtest
    gtest_main
    fast_collision_detection
    apollo_common 
    absl_strings
    absl_base
    absl_throw_delegate
    absl_str_format_internal
)

