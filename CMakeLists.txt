cmake_minimum_required(VERSION 3.16)

project(cpp_robot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(robot_test
    src/main.cpp
    src/dashboard/dashboard_server.cpp
    src/hal/gpio.cpp
    src/robot/motor_controller.cpp
    src/robot/robot_state.cpp
    src/telemetry/telemetry.cpp
)

target_include_directories(robot_test PRIVATE
    include
)

if(WIN32)
    target_link_libraries(robot_test ws2_32)
else()
    target_link_libraries(robot_test pthread)
endif()
