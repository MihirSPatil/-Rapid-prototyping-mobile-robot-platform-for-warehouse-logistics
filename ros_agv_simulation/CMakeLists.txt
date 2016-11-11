cmake_minimum_required(VERSION 2.8.3)
project(ros_agv_simulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
