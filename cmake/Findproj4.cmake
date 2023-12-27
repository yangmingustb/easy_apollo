# Configure PROJ4
#
# Set
#  PROJ4_FOUND = 1
#  PROJ4_INCLUDE_DIRS = /usr/local/include
#  PROJ4_LIBRARIES = proj
#  PROJ4_LIBRARY_DIRS = /usr/local/lib
#  PROJ4_BINARY_DIRS = /usr/local/bin
#  PROJ4_VERSION = 4.9.1 (for example)

message (STATUS "Reading ${CMAKE_CURRENT_LIST_FILE}")
# PROJ4_VERSION is set by version file
message (STATUS
  "PROJ4 configuration, version ${PROJ4_VERSION}")

# Tell the user project where to find our headers and libraries

set(proj4_INCLUDE_DIRS ${CMAKE_HOME_DIRECTORY}/third_party/install/proj4/include)
set(LIB_DIR ${CMAKE_HOME_DIRECTORY}/third_party/install/proj4/lib)
set(proj4_PREFIX ${CMAKE_HOME_DIRECTORY}/third_party/install/proj4)

set (PROJ4_INCLUDE_DIRS "${proj4_INCLUDE_DIRS}")
set (PROJ4_LIBRARY_DIRS "${LIB_DIR}")
set (PROJ4_BINARY_DIRS "${proj4_PREFIX}/bin")

set (PROJ4_LIBRARIES proj)
# Read in the exported definition of the library
include ("${proj4_PREFIX}/lib/cmake/proj4/proj4-targets.cmake")
include ("${proj4_PREFIX}/lib/cmake/proj4/proj4-namespace-targets.cmake")


# For backward compatibility with old releases of libgeotiff
set (PROJ4_INCLUDE_DIR ${PROJ4_INCLUDE_DIRS})
