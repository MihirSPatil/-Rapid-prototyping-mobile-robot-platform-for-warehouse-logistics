
add_executable(${PROJECT_NAME}_motion_generator motion_generator_node)
add_dependencies(${PROJECT_NAME}_motion_generator grizzly_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}_motion_generator ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_generator
                      PROPERTIES OUTPUT_NAME motion_generator PREFIX "")

add_executable(${PROJECT_NAME}_motion_safety motion_safety_node encoders_monitor motors_monitor)
add_dependencies(${PROJECT_NAME}_motion_safety grizzly_msgs_gencpp roboteq_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}_motion_safety ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_safety
                      PROPERTIES OUTPUT_NAME motion_safety PREFIX "")

add_executable(${PROJECT_NAME}_dead_reckoning dead_reckoning_node dead_reckoning)
add_dependencies(${PROJECT_NAME}_dead_reckoning grizzly_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}_dead_reckoning ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_dead_reckoning
                      PROPERTIES OUTPUT_NAME dead_reckoning PREFIX "")

install(TARGETS
  ${PROJECT_NAME}_dead_reckoning
  ${PROJECT_NAME}_motion_generator
  ${PROJECT_NAME}_motion_safety
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

