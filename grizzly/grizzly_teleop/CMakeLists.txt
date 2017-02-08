cmake_minimum_required(VERSION 2.8.3)
project(grizzly_teleop)

find_package(catkin)

catkin_package()

add_executable(_sixpair src/sixpair.c)
target_link_libraries(_sixpair usb)

install(TARGETS 
  _sixpair
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/teleop
  scripts/teleop_incremental
  scripts/sixpair
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



