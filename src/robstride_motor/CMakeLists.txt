cmake_minimum_required(VERSION 3.8)
project(robstride_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(robstride_motor_node
  src/robstride_motor_node.cpp
)

ament_target_dependencies(robstride_motor_node
  rclcpp
  std_msgs
  diagnostic_msgs
  std_srvs
)

target_include_directories(robstride_motor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS robstride_motor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robstride_motor test/test_robstride_motor.cpp)
  target_include_directories(test_robstride_motor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_node_safety test/test_node_safety.cpp)
  ament_target_dependencies(test_node_safety rclcpp std_msgs diagnostic_msgs std_srvs)
  target_include_directories(test_node_safety PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_add_test(test_find_motor_id
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/test_find_motor_id.sh"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 30
  )
endif()

ament_package()
