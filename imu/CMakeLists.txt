cmake_minimum_required(VERSION 3.5)
project(imu)

set(default_imu "mpu6050uart")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(${default_imu} ./src/mpu6050uart.cpp)
ament_target_dependencies(${default_imu} rclcpp sensor_msgs)

install(TARGETS ${default_imu}
        DESTINATION lib/${PROJECT_NAME})

ament_package()
