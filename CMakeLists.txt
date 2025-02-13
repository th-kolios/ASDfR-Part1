cmake_minimum_required(VERSION 3.8)
project(tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(tracker src/tracker.cpp)

# Link dependencies
ament_target_dependencies(tracker rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

# Install executable
install(TARGETS tracker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
