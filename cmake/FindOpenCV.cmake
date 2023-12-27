## Since opencv3.x, the cvRound has been moved to core/fastmath.hpp. The c interface
#  of cvRound is not availabe anymore.
#  see https://stackoverflow.com/questions/48365991/is-there-a-working-c-interface-for-opencv-3-x for details


# Find OpenCV_third_party/install cmake module sets the following variables:
# OpenCV_FOUND
# OpenCV_INCLUDE_DIRS
# OpenCV_LIBRARIES
# OpenCV_VERSION
# OpenCV_PREFIX  install directory

set(OpenCV_VERSION 4.2.0)
set(OpenCV_VERSION_MAJOR  4)
set(OpenCV_VERSION_MINOR  4)
set(OpenCV_VERSION_PATCH  0)
set(OpenCV_VERSION_TWEAK  0)
set(OpenCV_SHARED ON)


# x86
set(OpenCV_INCLUDE_DIRS ${CMAKE_HOME_DIRECTORY}/third_party/install/opencv/include/opencv4)
set(LIB_DIR ${CMAKE_HOME_DIRECTORY}/third_party/install/opencv/lib)
set(OpenCV_PREFIX ${CMAKE_HOME_DIRECTORY}/third_party/install/opencv)
# modules
# opencv_aruco opencv_calib3d opencv_core opencv_dnn opencv_features2d
# opencv_flann opencv_hdf opencv_highgui  opencv_imgcodecs opencv_imgproc
# opencv_ml opencv_objdetect opencv_photo opencv_shape opencv_stitching
# opencv_superres opencv_video opencv_videoio opencv_videostab
#
set(OpenCV_LIBRARIES 
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_calib3d${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_core${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_dnn${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_features2d${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_flann${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_highgui${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_imgcodecs${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_imgproc${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_ml${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_objdetect${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_photo${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_stitching${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_video${CMAKE_SHARED_LIBRARY_SUFFIX}
                      ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_videoio${CMAKE_SHARED_LIBRARY_SUFFIX}
                      # ${LIB_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}opencv_viz${CMAKE_SHARED_LIBRARY_SUFFIX}
                      )


set(OpenCV_FOUND true)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenCV DEFAULT_MSG
    OpenCV_LIBRARIES OpenCV_INCLUDE_DIRS)
mark_as_advanced(OpenCV_INCLUDE_DIRS OpenCV_LIBRARIES OpenCV_FOUND)

if(${OpenCV_FOUND})
    message(STATUS "Found OpenCV-${OpenCV_VERSION} in: " ${OpenCV_PREFIX})
    message(STATUS "opencv_LIBRARIES: ${OpenCV_LIBRARIES}")
    message(STATUS "opencv_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
else()
  message(SEND_ERROR "Could not find OpenCV")
endif()