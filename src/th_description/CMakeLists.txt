cmake_minimum_required(VERSION 3.8)
project(th_description)

find_package(ament_cmake REQUIRED)

# Installs share/th_description/robots/<robot>/{urdf,mjcf,meshes}, so URDF meshes resolve as
# package://th_description/robots/<robot>/meshes/... (robot_state_publisher, Foxglove, RViz).
install(DIRECTORY robots DESTINATION share/${PROJECT_NAME})

ament_package()
