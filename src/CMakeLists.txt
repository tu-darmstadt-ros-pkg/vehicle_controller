include_directories(${catkin_INCLUDE_DIRS} include)
add_executable(controller_node controller.cpp ps3d.cpp differential_drive_controller.cpp stuck_detector.cpp four_wheel_steer_controller.cpp)
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} monstertruck_msgs_generate_messages_cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES})
