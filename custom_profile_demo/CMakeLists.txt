add_executable(custom_profile_demo
  main.cpp
)

target_link_libraries(custom_profile_demo PRIVATE lxmaster::ecnet Threads::Threads)
set_target_properties(custom_profile_demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
