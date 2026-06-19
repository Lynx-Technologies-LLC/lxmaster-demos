add_executable(servo_mode_sweep_demo
  main.cpp
)

target_link_libraries(servo_mode_sweep_demo PRIVATE lxmstr::ecnet Threads::Threads)
set_target_properties(servo_mode_sweep_demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
