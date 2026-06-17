add_executable(servo_sin_vel_demo
  main.cpp
)

target_link_libraries(servo_sin_vel_demo PRIVATE lxmstr::ecnet Threads::Threads)

target_include_directories(servo_sin_vel_demo PRIVATE ${CMAKE_SOURCE_DIR}/common/include)

set_target_properties(servo_sin_vel_demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
