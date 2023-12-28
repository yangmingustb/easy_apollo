cmake_minimum_required(VERSION 3.5)
project(easy_apollo)

#amba
#SET(AMBA_PLATFORM 0)

#设定编译参数
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Debug")


set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -std=c++17  -pthread -fPIC")    # c++17
# set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -g")                     # 调试信息
#set(CMAKE_CXX_FLAGS   "-Wall")                  # 开启所有警告
# set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS} -O0" )             # 调试包不优化
#set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG " )   # release包优化####

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#message("current source: " ${CMAKE_CURRENT_SOURCE_DIR})
# include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/utils.cmake)

set(CMAKE_INSTALL_PREFIX "${CMAKE_HOME_DIRECTORY}/build")

#########################第三方库##########################
set(THIRD_LIB_DIR ${CMAKE_HOME_DIRECTORY}/third_party/install)

#openmp
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Poco REQUIRED COMPONENTS Foundation)
# find_package(fastrtps REQUIRED)
# find_package(fastcdr REQUIRED)

find_package(Protobuf REQUIRED)
# include_directories(${THIRD_LIB_DIR}/protobuf/include)
include_directories(${PROTOBUF_INCLUDE_DIRS})
# link_directories(${THIRD_LIB_DIR}/protobuf/lib)


#absl
#set(absl_DIR ${THIRD_LIB_DIR}/absl/lib/cmake/absl)
#find_package(absl REQUIRED)
include_directories(${THIRD_LIB_DIR}/abseil-cpp/include)
link_directories(${THIRD_LIB_DIR}/abseil-cpp/lib)
# set(DEP_LIBS ${DEP_LIBS} absl_strings absl_base absl_throw_delegate absl_str_format_internal)

# eigen
include_directories(${THIRD_LIB_DIR}/eigen-git-mirror/include/eigen3)
link_directories(${THIRD_LIB_DIR}/eigen-git-mirror/share/eigen3/cmake)
set(eigen_dir ${THIRD_LIB_DIR}/eigen-git-mirror/share/eigen3/cmake)
# find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
message("eigen: ${EIGEN3_INCLUDE_DIRS}")

#osqp qdldl
#set(osqp_DIR ${THIRD_LIB_DIR}/osqp/lib/cmake/osqp)
#find_package(osqp REQUIRED)
#set(qdldl_DIR ${THIRD_LIB_DIR}/osqp/lib/cmake/qdldl)
#find_package(qdldl REQUIRED)
include_directories(${THIRD_LIB_DIR}/osqp/include)
link_directories(${THIRD_LIB_DIR}/osqp/lib)
# set(DEP_LIBS ${DEP_LIBS} osqp qdldl)

#nlohmann_json
include_directories(${THIRD_LIB_DIR}/nlohmann_json/include)

#fastrtps
# include_directories(${THIRD_LIB_DIR}/fastrtps/include)
# include_directories(${THIRD_LIB_DIR}/Fast-CDR/include)
# link_directories(${THIRD_LIB_DIR}/fastrtps/lib)
# link_directories(${THIRD_LIB_DIR}/Fast-CDR/lib)
# set(DEP_LIBS ${DEP_LIBS} fastrtps fastcdr tinyxml2)

#gflags
include_directories(${THIRD_LIB_DIR}/gflags/include)
link_directories(${THIRD_LIB_DIR}/gflags/lib)
# set(DEP_LIBS ${DEP_LIBS} gflags)
# pkg_check_modules(Gflags REQUIRED gflags)

#glog
include_directories(${THIRD_LIB_DIR}/glog/include)
link_directories(${THIRD_LIB_DIR}/glog/lib)
# set(DEP_LIBS ${DEP_LIBS} glog)
find_package(glog REQUIRED)

#gtest
# find_package(gtest REQUIRED)
include_directories(${THIRD_LIB_DIR}/googletest/include)
link_directories(${THIRD_LIB_DIR}/googletest/lib)

#torch
set(Torch_DIR ${THIRD_LIB_DIR}/libtorch_cpu/share/cmake/Torch)
find_package(Torch)
include_directories(${TORCH_INCLUDE_DIRS})
link_directories(${THIRD_LIB_DIR}/libtorch_cpu/lib)
#set(DEP_LIBS ${DEP_LIBS} torch)
# set(DEP_LIBS ${DEP_LIBS} #torch_cpu # torch_global_deps torch)

message(STATUS "Torch_LIBRARIES: ${TORCH_LIBRARIES}")
message(STATUS "Torch_INCLUDE_DIRS: ${TORCH_INCLUDE_DIRS}")

# ubuntu 1804
include_directories(/usr/include/python3.6)

#ubuntu 20.04
# include_directories(/usr/include/python3.8)

#ad_rss_lib
include_directories(${THIRD_LIB_DIR}/ad_rss_lib/include)
include_directories(${THIRD_LIB_DIR}/ad_rss_lib/include/ad_rss)
link_directories(${THIRD_LIB_DIR}/ad_rss_lib/lib)
# set(DEP_LIBS ${DEP_LIBS} ad-rss)
#set(ad-rss-lib_DIR ${THIRD_LIB_DIR}/ad_rss_lib/lib/cmake/ad-rss-lib)
#find_package(ad-rss-lib)
#include_directories(${AD-RSS-LIB_INCLUDE_DIRS})
#message(STATUS "ad-rss-lib_INCLUDEDIR: ${AD-RSS-LIB_INCLUDE_DIRS}")

# set(DEP_LIBS ${DEP_LIBS} ipopt boost_system boost_filesystem boost_thread
#         ColPack
#         adolc
#         sqlite3
#         gtest
# )


# opencv
# include_directories(${install_path}/opencv/share/OpenCV)
find_package(OpenCV)
# include_directories(${install_path}/opencv/include)
# link_directories(${install_path}/opencv/lib)
include_directories(${OpenCV_INCLUDE_DIRS})

message(STATUS "Found OpenCV-${OpenCV_VERSION} in: " ${OpenCV_VERSION})
message(STATUS "opencv_LIBRARIES: ${OpenCV_LIBRARIES}")
message(STATUS "opencv_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")


# include_directories(${THIRD_LIB_DIR}/proj4/include)
# link_directories(${THIRD_LIB_DIR}/proj4/lib)
# find_package(proj4)
# include_directories(${PROJ4_INCLUDE_DIRS})

# message(STATUS "Found proj4 version in: " ${PROJ4_VERSION})
# message(STATUS "PROJ4_LIBRARY_DIRS: ${PROJ4_LIBRARY_DIRS}")
# message(STATUS "PROJ4_LIBRARY: ${PROJ4_LIBRARIES}")
# message(STATUS "PROJ4_INCLUDE_DIRS: ${PROJ4_INCLUDE_DIRS}")



# tf2
include_directories(${THIRD_LIB_DIR}/tf2/include)
link_directories(${THIRD_LIB_DIR}/tf2/lib)


include(GNUInstallDirs)


# path
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
  )


add_subdirectory(cyber)
add_subdirectory(modules)

add_library(apollo INTERFACE)
target_link_libraries(apollo INTERFACE
                apollo_control
                apollo_planning
                
                localization_common
                localization_proto

                apollo_perception
                apollo_map
                apollo_routing
                apollo_prediction
                apollo_common
                apollo_canbus
                apollo_storytelling
        )



add_subdirectory(test)


configure_file("setup.bash.in" "${CMAKE_BINARY_DIR}/setup.bash" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/setup.bash
  DESTINATION .
)

configure_file("set_env.bash" "${CMAKE_BINARY_DIR}/set_env.bash" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/set_env.bash
  DESTINATION .
)

configure_file("set_python_env.bash" "${CMAKE_HOME_DIRECTORY}/set_python_env.bash" @ONLY)
install(FILES ${CMAKE_HOME_DIRECTORY}/set_python_env.bash
  DESTINATION .
)

configure_file("launch_real.sh" "${CMAKE_BINARY_DIR}/launch_real.bash" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/launch_real.bash
  DESTINATION .
)

configure_file("launch_simulation.sh" "${CMAKE_BINARY_DIR}/launch_simulation.sh" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/launch_simulation.sh
  DESTINATION .
)

configure_file("stop_all.sh" "${CMAKE_BINARY_DIR}/stop_all.bash" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/stop_all.bash
  DESTINATION .
)

configure_file("x86_compile.sh" "${CMAKE_BINARY_DIR}/x86_compile.sh" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/x86_compile.sh
  DESTINATION .
)

configure_file("launch_replay.sh" "${CMAKE_BINARY_DIR}/launch_replay.sh" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/launch_replay.sh
  DESTINATION .
)

configure_file("launch_record.sh" "${CMAKE_BINARY_DIR}/launch_record.sh" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/launch_record.sh
  DESTINATION .
)

configure_file("launch_replay_apollo.sh" "${CMAKE_BINARY_DIR}/launch_replay_apollo.sh" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/launch_replay_apollo.sh
  DESTINATION .
)

configure_file("launch_cyber_monitor.sh" "${CMAKE_BINARY_DIR}/launch_cyber_monitor.sh" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/launch_cyber_monitor.sh
  DESTINATION .
)

configure_file("launch_dreamview.sh" "${CMAKE_BINARY_DIR}/launch_dreamview.sh" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/launch_dreamview.sh
  DESTINATION .
)


