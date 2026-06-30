cmake_minimum_required(VERSION 3.16)
project(lxmaster_demos CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Consumes an installed LXMASTER package. This provides the imported target
# lxmaster::ecnet (which transitively carries facade/devices/ecmaster/eni and the
# public headers, including the <lxmaster/lxmaster.hpp> umbrella).
find_package(lxmaster CONFIG REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(custom_profile_demo)
add_subdirectory(do_shift_demo)
add_subdirectory(servo_mode_sweep_demo)
add_subdirectory(servo_sin_demo)
add_subdirectory(servo_sin_torque_demo)
add_subdirectory(servo_sin_vel_demo)
add_subdirectory(sine_shift_demo)
