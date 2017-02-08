catkin_add_gtest(${PROJECT_NAME}_dead_reckoning_tests dead_reckoning_tests ../src/dead_reckoning)
target_link_libraries(${PROJECT_NAME}_dead_reckoning_tests ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}_encoders_monitor_tests encoders_monitor_tests ../src/encoders_monitor)
target_link_libraries(${PROJECT_NAME}_encoders_monitor_tests ${catkin_LIBRARIES})
