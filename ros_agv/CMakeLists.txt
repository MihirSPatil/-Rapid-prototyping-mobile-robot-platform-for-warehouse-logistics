cmake_minimum_required(VERSION 2.8.3)
project(ros_agv)

find_package(catkin REQUIRED COMPONENTS
  ros_agv_base
  ros_agv_bringup
  ros_agv_desc
  ros_agv_simulation
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
