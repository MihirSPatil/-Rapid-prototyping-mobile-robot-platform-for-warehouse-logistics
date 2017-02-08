cmake_minimum_required(VERSION 2.8.3)
project(grizzly_description)

find_package(catkin)

catkin_package()

install(
  DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

