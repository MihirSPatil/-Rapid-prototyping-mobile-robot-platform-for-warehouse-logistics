cmake_minimum_required(VERSION 2.8.3)
project(ros_agv_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_arduino
  rosserial_client
  rosserial_python
  rosserial_server
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
