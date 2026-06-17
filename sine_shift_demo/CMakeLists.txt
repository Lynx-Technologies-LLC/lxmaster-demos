add_executable(sine_shift_demo
  main.cpp
)

target_link_libraries(sine_shift_demo PRIVATE lxmstr::ecnet Threads::Threads)

target_include_directories(sine_shift_demo PRIVATE ${CMAKE_SOURCE_DIR}/common/include)

set_target_properties(sine_shift_demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
