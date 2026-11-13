cmake_minimum_required(VERSION 3.8)
project(th_policy_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(policy_bridge src/policy_bridge.cpp)
target_compile_features(policy_bridge PRIVATE cxx_std_17)
target_include_directories(policy_bridge PRIVATE include)
ament_target_dependencies(policy_bridge rclcpp geometry_msgs sensor_msgs std_msgs)

install(TARGETS policy_bridge DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bridge_math test/test_bridge_math.cpp)
  target_compile_features(test_bridge_math PRIVATE cxx_std_17)
  target_include_directories(test_bridge_math PRIVATE include)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_pipeline test/test_pipeline.py
    ENV POLICY_BRIDGE_EXECUTABLE=$<TARGET_FILE:policy_bridge>
    TIMEOUT 120)
endif()

ament_package()
