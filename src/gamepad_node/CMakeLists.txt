cmake_minimum_required(VERSION 3.8)
project(gamepad_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(gamepad_node src/gamepad_node.cpp)
target_include_directories(gamepad_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(gamepad_node PRIVATE cxx_std_17)
ament_target_dependencies(gamepad_node rclcpp sensor_msgs geometry_msgs std_msgs)

install(TARGETS gamepad_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_teleop_mapping test/test_teleop_mapping.cpp)
  target_include_directories(test_teleop_mapping PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_teleop_mapping PRIVATE cxx_std_17)
endif()

ament_package()
