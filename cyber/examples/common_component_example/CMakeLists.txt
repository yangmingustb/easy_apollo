add_executable(channel_test_writer channel_test_writer.cc ../proto/examples.pb.cc)
target_link_libraries(channel_test_writer
    cyber
)

add_executable(channel_prediction_writer channel_prediction_writer.cc ../proto/examples.pb.cc)
target_link_libraries(channel_prediction_writer
    cyber
)

add_library(common_component_example SHARED common_component_example.cc ../proto/examples.pb.cc)
target_link_libraries(common_component_example
  cyber
)
set_target_properties(common_component_example PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/")


file(COPY common.dag common.launch
     DESTINATION ${CMAKE_BINARY_DIR}/share/examples  
)

# install(FILES common.dag common.launch
#   DESTINATION share/examples/
# )

# install(TARGETS common_component_example
#     LIBRARY DESTINATION lib
# )

