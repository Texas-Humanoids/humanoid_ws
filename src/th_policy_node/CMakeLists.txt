cmake_minimum_required(VERSION 3.8)
project(th_policy_node)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h REQUIRED)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime REQUIRED)

add_executable(policy_node src/policy_node.cpp)
target_compile_features(policy_node PRIVATE cxx_std_17)
target_include_directories(policy_node PRIVATE include)
ament_target_dependencies(policy_node rclcpp sensor_msgs std_msgs)
target_include_directories(policy_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(policy_node ${ONNXRUNTIME_LIBRARY})

install(TARGETS policy_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_policy_profile test/test_policy_profile.cpp)
  target_compile_features(test_policy_profile PRIVATE cxx_std_17)
  target_include_directories(test_policy_profile PRIVATE include)
endif()

ament_package()
