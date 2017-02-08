cmake_minimum_required(VERSION 2.8.3)
project(grizzly_msgs)

find_package(catkin REQUIRED COMPONENTS cmake_modules message_generation std_msgs)
find_package(Eigen REQUIRED)

add_message_files(
  FILES
  Ambience.msg
  Drive.msg
  RawStatus.msg
)
generate_messages(
  DEPENDENCIES std_msgs  
)
catkin_package(
  CATKIN_DEPENDS message_runtime
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} 
  DEPENDS eigen
)

# Install the helper header of Eigen/msg conversions.
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
