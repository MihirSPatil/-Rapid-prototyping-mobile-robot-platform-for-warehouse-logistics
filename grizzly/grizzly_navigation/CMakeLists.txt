cmake_minimum_required(VERSION 2.8.3)
project(grizzly_navigation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roslaunch)

catkin_package()
roslaunch_add_file_check(launch/localization.launch)

#############
## Install ##
#############
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

