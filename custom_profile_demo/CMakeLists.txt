add_executable(custom_profile_demo
  main.cpp
)

# Find the demo's own header (a6_extended_profile.hpp) alongside main.cpp.
target_include_directories(custom_profile_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(custom_profile_demo PRIVATE lxmaster::ecnet Threads::Threads)
set_target_properties(custom_profile_demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
