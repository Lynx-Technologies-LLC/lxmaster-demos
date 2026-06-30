cmake_minimum_required(VERSION 3.16)
project(lxmaster_demos CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# LXMASTER is an RT system and is RTTI-free on the device path: recover a custom profile type with a
# profileName()-guarded static_cast (once at setup), never dynamic_cast/typeid. Build the demos with
# -fno-rtti so any reintroduced dynamic_cast/typeid fails to compile. CXX-scoped; toggle with
# -DLXMASTER_DEMOS_NO_RTTI=OFF if a demo ever genuinely needs RTTI.
option(LXMASTER_DEMOS_NO_RTTI "Compile demos with -fno-rtti (dynamic_cast/typeid will not build)" ON)
if(LXMASTER_DEMOS_NO_RTTI)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
endif()

# Consumes an installed LXMASTER package. This provides the imported target
# lxmaster::ecnet (which transitively carries facade/devices/ecmaster/eni and the
# public headers, including the <lxmaster/lxmaster.hpp> umbrella).
find_package(lxmaster CONFIG REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(custom_profile_demo)
add_subdirectory(imu_demo)
add_subdirectory(do_shift_demo)
add_subdirectory(servo_mode_sweep_demo)
add_subdirectory(servo_sin_demo)
add_subdirectory(servo_sin_torque_demo)
add_subdirectory(servo_sin_vel_demo)
add_subdirectory(sine_shift_demo)
