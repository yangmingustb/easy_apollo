cmake_minimum_required(VERSION 3.5.1)

project(cyber VERSION 7.0.0)

set(TARGET_NAME ${PROJECT_NAME})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_language(ASM)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(CROUTINE_FILE "croutine/detail/swap_x86_64.S")
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(CROUTINE_FILE "croutine/detail/swap_x86_64.S")
  message("### SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR} ${CROUTINE_FILE}")
else()
  set(CROUTINE_FILE "croutine/detail/swap_aarch64.S")
  message("### SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR} ${CROUTINE_FILE}")
endif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

find_package(fastrtps)

# set(FASTRTPS_DIR ${CMAKE_HOME_DIRECTORY}/third_party/install)
set(THIRD_LIB_DIR ${CMAKE_HOME_DIRECTORY}/third_party/install)

# include_directories(${FASTRTPS_DIR}/fastrtps/include)
# link_directories(${FASTRTPS_DIR}/fastrtps/lib)

# find_package(fastrtps REQUIRED)
# find_package(fastcdr REQUIRED)

find_package(Poco REQUIRED COMPONENTS Foundation)

#gflags
include_directories(${THIRD_LIB_DIR}/gflags/include)
link_directories(${THIRD_LIB_DIR}/gflags/lib)

#glog
# include_directories(${THIRD_LIB_DIR}/glog/include)
# link_directories(${THIRD_LIB_DIR}/glog/lib)
# set(DEP_LIBS ${DEP_LIBS} glog)

find_package(glog REQUIRED)
# find_package(Python REQUIRED COMPONENTS Interpreter Development)
# pkg_check_modules(Python REQUIRED python3)
find_package(Protobuf REQUIRED)

# pkg_check_modules(Gflags REQUIRED gflags)

set(Python_VERSION "${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}")
set(Python_VERSION "${Python_VERSION}")
set(CYBER_PYTHON_PATH "lib/python${Python_VERSION}/site-packages/cyber/python")
set(CYBER_PYTHON_INTERNAL_PATH "${CYBER_PYTHON_PATH}/internal")
set(CYBER_PYTHON_PROTOBUF_PATH "lib/python${Python_VERSION}/site-packages/cyber/proto")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    # ${fastcdr_INCLUDE_DIR}
    ${fastrtps_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    # ${Python_INCLUDE_DIRS}
    ${Glog_INCLUDE_DIRS}
    ${Gflags_INCLUDE_DIRS}
)

link_directories(
    # ${fastcdr_LIB_DIR}
    ${fastrtps_LIB_DIRS}
    ${Protobuf_LIBRARIES_DIRS}
    ${Glog_LIBRARY_DIRS}
    ${Gflags_LIBRARY_DIRS}
)

file(GLOB CYBER_SRCS 
    "base/*.cc"
    "blocker/*.cc"
    "class_loader/*.cc"
    "class_loader/*/*.cc"
    "common/*.cc"
    "component/*.cc"
    "croutine/*.cc"
    "croutine/*/*.cc"
    "data/*.cc"
    "event/*.cc"
    "io/*.cc"
    "logger/*.cc"
    "message/*.cc"
    "node/*.cc"
    "parameter/*.cc"
    "plugin_manager/*.cc"
    "record/*.cc"
    "record/*/*.cc"
    "scheduler/*.cc"
    "scheduler/*/*.cc"
    "service/*.cc"
    "service_discovery/*.cc"
    "service_discovery/*/*.cc"
    "task/*.cc"
    "time/*.cc"
    "timer/*.cc"
    "transport/*.cc"
    "transport/*/*.cc"
    "*.cc"
    "sysmo/*.cc"
    ${CROUTINE_FILE}
)

# file(GLOB CYBER_PROTO_FILES
#   "${CMAKE_CURRENT_SOURCE_DIR}/cyber/proto/*.proto"
#   "${CMAKE_CURRENT_SOURCE_DIR}/cyber/examples/proto/*.proto"
# )
# SET(PROTO_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})
# LIST(APPEND PROTO_FLAGS
#  -I${CMAKE_CURRENT_SOURCE_DIR}
# )
# FOREACH(FIL ${CYBER_PROTO_FILES})
#   GET_FILENAME_COMPONENT(FIL_WE ${FIL} NAME_WE)
#   string(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${FIL})
#   string(REGEX REPLACE "(.+)\\${FILE_NAME}.*" "\\1" FILE_PATH ${FIL})
#   string(REGEX MATCH "(/cyber/proto/.*|/cyber/examples/proto/.*)" OUT_PATH ${FILE_PATH})
#   set(PROTO_SRCS "${CMAKE_CURRENT_BINARY_DIR}${OUT_PATH}${FIL_WE}.pb.cc")
#   set(PROTO_HDRS "${CMAKE_CURRENT_BINARY_DIR}${OUT_PATH}${FIL_WE}.pb.h")
#   EXECUTE_PROCESS(
#     COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${PROTO_FLAGS} --cpp_out=${PROTO_META_BASE_DIR} ${FIL}
#   )
#   file(COPY ${PROTO_SRCS} DESTINATION ${FILE_PATH})
#   file(COPY ${PROTO_HDRS} DESTINATION ${FILE_PATH})
#   LIST(APPEND ALL_PROTO_SRCS ${PROTO_SRCS})
#   LIST(APPEND ALL_PROTO_HDRS ${PROTO_HDRS})
# ENDFOREACH()

aux_source_directory("./proto" CYBER_PROTO_SRCS)

list(FILTER CYBER_SRCS EXCLUDE REGEX .*test[.]cc)

message("==========================")
message("cyber source: ${CYBER_PROTO_SRCS}")

add_library(cyber SHARED
  ${CYBER_PROTO_SRCS}
  # ${ALL_PROTO_SRCS}
  ${CYBER_SRCS}
)
target_include_directories(cyber PUBLIC
                                  ${fastrtps_INCLUDE_DIRS}
)

target_link_libraries(cyber PUBLIC
    ${Glog_LIBRARIES}
    ${Gflags_LIBRARIES}
    ${Protobuf_LIBRARIES}
    ${Poco_LIBRARIES}
    glog
    ${fastrtps_LIBRARIES}
    
    # fastcdr
    atomic
    uuid
    gflags
    tinyxml2
    # dl
    # rt
)


add_executable(mainboard
  mainboard/mainboard.cc
  mainboard/module_argument.cc
  mainboard/module_controller.cc
)
target_link_libraries(mainboard
  pthread
  cyber
)
set_target_properties(mainboard PROPERTIES RUNTIME_OUTPUT_DIRECTORY "bin/")


add_subdirectory(examples)
add_subdirectory(python)
add_subdirectory(tools)

# # --------------------------------------------------------------
# #                      CPack
# # --------------------------------------------------------------
# # 打包方式 
# include(InstallRequiredSystemLibraries)
# if (APPLE)
# elseif (UNIX)
#   set (CPACK_GENERATOR        "DEB")
#   set (CPACK_SOURCE_GENERATOR "TGZ;ZIP")
# endif ()

# # system/architecture
# if (WINDOWS)
#   if (CMAKE_CL_64)
#     set (CPACK_SYSTEM_NAME "win64")
#   else ()
#     set (CPACK_SYSTEM_NAME "win32")
#   endif ()
#   set (CPACK_PACKAGE_ARCHITECTURE)
# elseif (APPLE)
#   set (CPACK_PACKAGE_ARCHITECTURE "darwin")
# else ()
#   string (TOLOWER "${CMAKE_SYSTEM_NAME}" CPACK_SYSTEM_NAME)
#   if (CMAKE_CXX_FLAGS MATCHES "-m32")
#     set (CPACK_PACKAGE_ARCHITECTURE i386)
#   else ()
#     execute_process (
#       COMMAND         dpkg --print-architecture
#       RESULT_VARIABLE RV
#       OUTPUT_VARIABLE CPACK_PACKAGE_ARCHITECTURE
#       )
#     if (RV EQUAL 0)
#           string (STRIP "${CPACK_PACKAGE_ARCHITECTURE}" CPACK_PACKAGE_ARCHITECTURE)
#     else ()
#       execute_process (COMMAND uname -m OUTPUT_VARIABLE CPACK_PACKAGE_ARCHITECTURE)
#       if (CPACK_PACKAGE_ARCHITECTURE MATCHES "x86_64")
#             set (CPACK_PACKAGE_ARCHITECTURE amd64)
#       else ()
#         set (CPACK_PACKAGE_ARCHITECTURE i386)
#       endif ()
#     endif ()
#   endif ()
# endif ()

# set(CPACK_PACKAGE_NAME   "cyber")
# set(CPACK_PACKAGE_CONTACT "job@minhang.me")
# set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
# set(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
# set(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
# set(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}")
# set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Apollo CyberRT")
# set(CPACK_INSTALL_PREFIX              "${CMAKE_INSTALL_PREFIX}")
# set(CPACK_OUTPUT_FILE_PREFIX          "packages")
# set(CPACK_PACKAGE_RELOCATABLE         TRUE)
# set(CPACK_MONOLITHIC_INSTALL          TRUE)
# set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
# set(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
# set(CPACK_RPM_PACKAGE_LICENSE "MIT")
# set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/minhanghuang/CyberRT")
# set(CPACK_PACKAGE_FILE_NAME   "lib${CPACK_PACKAGE_NAME}")
# set(CPACK_PACKAGE_FILE_NAME   "${CPACK_PACKAGE_FILE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_PACKAGE_ARCHITECTURE}")

# include(CPack)
# # --------------------------------------------------------------
# #                       package
# # --------------------------------------------------------------
# configure_file("cmake/cyber.pc.in"
#   "${PROJECT_BINARY_DIR}/pkgconfig/cyber.pc" @ONLY)

# configure_package_config_file(
#   "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cyber-config.cmake.in"
#   "cmake/cyber-config.cmake"
#   INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/cyber"
#   NO_CHECK_REQUIRED_COMPONENTS_MACRO
#   PATH_VARS CMAKE_INSTALL_INCLUDEDIR CMAKE_INSTALL_LIBDIR TARGET_NAME
# )

# write_basic_package_version_file(cmake/cyber-config-version.cmake
#   VERSION ${PROJECT_VERSION}
#   COMPATIBILITY AnyNewerVersion
# )

# install(FILES
#   "${PROJECT_BINARY_DIR}/pkgconfig/cyber.pc"
#   DESTINATION "${CMAKE_INSTALL_DATADIR}/pkgconfig"
# )

# install(FILES 
#   "${PROJECT_BINARY_DIR}/cmake/cyber-config.cmake"
#   "${PROJECT_BINARY_DIR}/cmake/cyber-config-version.cmake"
#   DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/cyber"
# )

# install(EXPORT cyber-targets
#   DESTINATION "${CMAKE_INSTALL_DATADIR}/cmake/cyber"
# )

# # cyber .h files
# install(DIRECTORY cyber/
#     DESTINATION include/cyber/
#     FILES_MATCHING
#     PATTERN "*.h"
# )

# # apollo conf
# file(COPY cyber/conf/
#   DESTINATION ${CMAKE_BINARY_DIR}/share/conf/
# )
# install(DIRECTORY ${CMAKE_BINARY_DIR}/share/conf/
#   DESTINATION share/conf/
# )

# # setup.zsh
# configure_file("setup.bash.in" "${CMAKE_BINARY_DIR}/setup.bash" @ONLY)
# configure_file("setup.zsh.in" "${CMAKE_BINARY_DIR}/setup.zsh" @ONLY)
# install(FILES ${CMAKE_BINARY_DIR}/setup.zsh ${CMAKE_BINARY_DIR}/setup.bash
#   DESTINATION .
# )

# cyber python api
file(COPY python/
  DESTINATION ${CMAKE_BINARY_DIR}/${CYBER_PYTHON_PATH}
)
install(DIRECTORY ${CMAKE_BINARY_DIR}/${CYBER_PYTHON_PATH}/
  DESTINATION ${CYBER_PYTHON_PATH}
  FILES_MATCHING
  PATTERN "*.py"
)

# cyber python protobuf
file(WRITE ${CMAKE_BINARY_DIR}/${CYBER_PYTHON_PROTOBUF_PATH}/__init__.py "import os\n")
file(APPEND ${CMAKE_BINARY_DIR}/${CYBER_PYTHON_PROTOBUF_PATH}/__init__.py "import sys\n")
file(APPEND ${CMAKE_BINARY_DIR}/${CYBER_PYTHON_PROTOBUF_PATH}/__init__.py "sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))\n")
file(COPY cyber/proto/
  DESTINATION ${CMAKE_BINARY_DIR}/${CYBER_PYTHON_PROTOBUF_PATH}
  FILES_MATCHING
  PATTERN "*.py"
)

install(DIRECTORY ${CMAKE_BINARY_DIR}/${CYBER_PYTHON_PROTOBUF_PATH}/
  DESTINATION ${CYBER_PYTHON_PROTOBUF_PATH}
)



install(TARGETS 
          cyber
          mainboard
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION .
        )

