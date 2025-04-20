cmake_minimum_required(VERSION 3.5)
project(jackal_control)


find_package(ament_cmake REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()