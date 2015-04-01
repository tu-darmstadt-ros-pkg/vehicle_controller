include_directories(${catkin_INCLUDE_DIRS} include)
add_executable(controller_node controller.cpp ps3d.cpp)
add_dependencies(controller_node monstertruck_msgs_generate_messages_cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES})
