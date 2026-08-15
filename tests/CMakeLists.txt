add_executable(capture_file_camera_replay_pacing_test replay_pacing_test.cpp)
add_executable(capture_file_camera_runtime_param_compat_test
               runtime_param_compat_test.cpp)

target_include_directories(
  capture_file_camera_replay_pacing_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..
                                                 ${OpenCV_INCLUDE_DIRS})
target_include_directories(
  capture_file_camera_runtime_param_compat_test
  PRIVATE ${CMAKE_CURRENT_LIST_DIR}/.. ${OpenCV_INCLUDE_DIRS})
target_link_libraries(capture_file_camera_replay_pacing_test
                      PRIVATE ${OpenCV_LIBS})
target_link_libraries(capture_file_camera_runtime_param_compat_test
                      PRIVATE xr ${OpenCV_LIBS})
target_compile_features(capture_file_camera_replay_pacing_test
                        PRIVATE cxx_std_20)
target_compile_features(capture_file_camera_runtime_param_compat_test
                        PRIVATE cxx_std_20)

add_test(NAME capture_file_camera_replay_pacing_test
         COMMAND capture_file_camera_replay_pacing_test)
add_test(NAME capture_file_camera_runtime_param_compat_test
         COMMAND capture_file_camera_runtime_param_compat_test)
