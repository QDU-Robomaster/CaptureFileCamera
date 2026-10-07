find_package(Threads REQUIRED)

add_executable(capture_file_recording_test recording_test.cpp)
add_executable(capture_file_camera_test capture_file_camera_test.cpp)

foreach(test capture_file_recording_test capture_file_camera_test)
  target_include_directories(${test} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
  target_link_libraries(${test} PRIVATE xr Threads::Threads)
  target_compile_features(${test} PRIVATE cxx_std_20)
  add_test(NAME ${test} COMMAND ${test})
endforeach()
set_tests_properties(capture_file_camera_test PROPERTIES TIMEOUT 30)
