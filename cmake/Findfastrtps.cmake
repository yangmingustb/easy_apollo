
set(fastrtps_version 1.5.0)


# x86
set(fastrtps_INCLUDE_DIRS ${CMAKE_HOME_DIRECTORY}/third_party/install/fastrtps/include)
set(fastrtps_LIB_DIRS ${CMAKE_HOME_DIRECTORY}/third_party/install/fastrtps/lib)
set(fastrtps_PREFIX ${CMAKE_HOME_DIRECTORY}/third_party/install/fastrtps)

#
set(fastrtps_LIBRARIES 
                      ${fastrtps_LIB_DIRS}/${CMAKE_SHARED_LIBRARY_PREFIX}fastcdr${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${fastrtps_LIB_DIRS}/${CMAKE_SHARED_LIBRARY_PREFIX}fastrtps${CMAKE_SHARED_LIBRARY_SUFFIX}
                      )


set(fastrtps_FOUND true)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(fastrtps DEFAULT_MSG
fastrtps_LIBRARIES fastrtps_INCLUDE_DIRS)
mark_as_advanced(fastrtps_INCLUDE_DIRS fastrtps_LIBRARIES fastrtps_FOUND)

if(${fastrtps_FOUND})
    message(STATUS "Found fastrtps-${fastrtps_version} in: " ${fastrtps_PREFIX})
    message(STATUS "fastrtps_LIBRARIES: ${fastrtps_LIBRARIES}")
    message(STATUS "fastrtps_INCLUDE_DIRS: ${fastrtps_INCLUDE_DIRS}")
else()
  message(SEND_ERROR "Could not find fastrtps")
endif()