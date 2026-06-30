add_executable(imu_demo
  main.cpp
)

target_link_libraries(imu_demo PRIVATE lxmaster::ecnet Threads::Threads)
set_target_properties(imu_demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
