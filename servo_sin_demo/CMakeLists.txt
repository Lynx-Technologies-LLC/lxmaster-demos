add_executable(servo_sin_demo
  main.cpp
)

target_link_libraries(servo_sin_demo PRIVATE lxmaster::ecnet Threads::Threads)
set_target_properties(servo_sin_demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
