cmake_minimum_required(VERSION 3.10)
project(th_basic_cv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# The CV pipeline and image conversions, shared by both nodes and the tests.
add_library(basic_cv_lib STATIC src/basic_cv.cpp src/image_conversions.cpp)
target_include_directories(basic_cv_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(basic_cv_lib PUBLIC
  ${OpenCV_LIBS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS})

add_executable(basic_cv_node src/basic_cv_node.cpp)
target_link_libraries(basic_cv_node basic_cv_lib rclcpp::rclcpp)

add_executable(sample_image_publisher src/sample_image_publisher.cpp)
target_link_libraries(sample_image_publisher
  basic_cv_lib rclcpp::rclcpp ament_index_cpp::ament_index_cpp)

install(TARGETS basic_cv_node sample_image_publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch sample_images
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # No per-file copyright headers yet; the repo LICENSE covers the code.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_basic_cv test/test_basic_cv.cpp)
  target_link_libraries(test_basic_cv basic_cv_lib)
  target_compile_definitions(test_basic_cv PRIVATE
    SAMPLE_IMAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/sample_images")
endif()

ament_package()
