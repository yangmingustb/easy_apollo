cmake_minimum_required(VERSION 3.5.1)

add_library(localization_common SHARED
                 localization_gflags.cc)
target_link_libraries(localization_common ${gflags_LIBRARIES})



install(TARGETS localization_common
        
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION .
        )
