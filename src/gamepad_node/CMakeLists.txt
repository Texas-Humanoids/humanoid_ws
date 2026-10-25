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
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Teach cppcheck the Google Test macros instead of treating them as C++ syntax.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(LIBRARIES googletest INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_input test/test_input.cpp)
  target_include_directories(test_input PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_input PRIVATE cxx_std_17)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gamepad_node test/test_gamepad_node.py
    ENV GAMEPAD_NODE_EXECUTABLE=$<TARGET_FILE:gamepad_node>
    TIMEOUT 120)
endif()

ament_package()
