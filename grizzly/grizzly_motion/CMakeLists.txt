cmake_minimum_required(VERSION 2.8.3)
project(grizzly_motion)

find_package(catkin REQUIRED COMPONENTS
   cmake_modules
   nmea_msgs
   geometry_msgs
   grizzly_msgs
   roboteq_msgs
   roscpp
   tf
   diagnostic_updater
   angles
 )

find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  nmea_msgs
  INCLUDE_DIRS include
  DEPENDS eigen
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_subdirectory(src)

add_executable(motion_generator src/motion_generator_node.cpp)
target_link_libraries(motion_generator ${catkin_LIBRARIES})
add_dependencies(motion_generator ${nmea_msgs_EXPORTED_TARGETS})

#add_executable(dead_reckoning src/dead_reckoning_node.cpp)
#target_link_libraries(dead_reckoning ${catkin_LIBRARIES})
#add_dependencies(dead_reckoning ${nmea_msgs_EXPORTED_TARGETS})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
