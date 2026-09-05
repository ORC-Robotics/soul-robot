cmake_minimum_required(VERSION 3.16)

project(cpp_robot)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(robot_test
    src/main.cpp
    src/dashboard/dashboard_server.cpp
    src/hal/camera_monitor.cpp
    src/hal/esp32_bridge.cpp
    src/hal/gpio.cpp
    src/hal/status_led.cpp
    src/robot/motor_controller.cpp
    src/robot/curve_diagnostics_logger.cpp
    src/robot/imu_turn_controller.cpp
    src/robot/main_mission.cpp
    src/robot/mission_controller.cpp
    src/robot/oled_event_notifier.cpp
    src/robot/robot_state.cpp
    src/robot/servo_controller.cpp
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

if(BUILD_TESTING)
    add_executable(main_mission_test
        tests/main_mission_test.cpp
        src/hal/camera_monitor.cpp
        src/robot/imu_turn_controller.cpp
        src/robot/main_mission.cpp
        src/robot/robot_state.cpp
    )
    target_include_directories(main_mission_test PRIVATE include)
    if(NOT WIN32)
        target_link_libraries(main_mission_test pthread)
    endif()
    add_test(NAME main_mission_test COMMAND main_mission_test)

    add_executable(curve_diagnostics_logger_test
        tests/curve_diagnostics_logger_test.cpp
        src/robot/curve_diagnostics_logger.cpp
    )
    target_include_directories(curve_diagnostics_logger_test PRIVATE include)
    if(NOT WIN32)
        target_link_libraries(curve_diagnostics_logger_test pthread)
    endif()
    add_test(
        NAME curve_diagnostics_logger_test
        COMMAND curve_diagnostics_logger_test)

    add_executable(servo_controller_test
        tests/servo_controller_test.cpp
        src/hal/esp32_bridge.cpp
        src/robot/robot_state.cpp
        src/robot/servo_controller.cpp
    )
    target_include_directories(servo_controller_test PRIVATE include)
    if(NOT WIN32)
        target_link_libraries(servo_controller_test pthread)
    endif()
    add_test(NAME servo_controller_test COMMAND servo_controller_test)

    add_executable(oled_event_notifier_test
        tests/oled_event_notifier_test.cpp
        src/hal/esp32_bridge.cpp
        src/robot/oled_event_notifier.cpp
    )
    target_include_directories(oled_event_notifier_test PRIVATE include)
    if(NOT WIN32)
        target_link_libraries(oled_event_notifier_test pthread)
    endif()
    add_test(NAME oled_event_notifier_test COMMAND oled_event_notifier_test)

endif()
