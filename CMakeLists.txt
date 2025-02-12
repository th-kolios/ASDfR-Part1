cmake_minimum_required(VERSION 3.8)
project(avg_brightness_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  ${cv_bridge_INCLUDE_DIRS}
)


add_executable(avg_brightness_node src/avg_brightness_node.cpp)

#Link dependencies
ament_target_dependencies(avg_brightness_node rclcpp sensor_msgs std_msgs cv_bridge)

#Install the executable
install(TARGETS avg_brightness_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
