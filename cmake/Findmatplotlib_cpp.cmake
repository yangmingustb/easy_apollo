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
  "matplotlib configuration, version ${PROJ4_VERSION}")

# Tell the user project where to find our headers and libraries

set(matplotlib_INCLUDE_DIRS ${CMAKE_HOME_DIRECTORY}/third_party/install/matplotlib-cpp/include)
set(LIB_DIR ${CMAKE_HOME_DIRECTORY}/third_party/install/matplotlib-cpp/lib)
set(matplotlib_PREFIX ${CMAKE_HOME_DIRECTORY}/third_party/install/matplotlib-cpp)

set (MATPLOTLIB_INCLUDE_DIRS "${matplotlib_INCLUDE_DIRS}")
set (MATPLOTLIB_LIBRARY_DIRS "${LIB_DIR}")



# For backward compatibility with old releases of libgeotiff
set (MATPLOTLIB_INCLUDE_DIR ${MATPLOTLIB_INCLUDE_DIRS})
