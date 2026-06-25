add_executable(sine_shift_demo
  main.cpp
)

target_link_libraries(sine_shift_demo PRIVATE lxmaster::ecnet Threads::Threads)
set_target_properties(sine_shift_demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
