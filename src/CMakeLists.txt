include_directories(${catkin_INCLUDE_DIRS} include)


include_directories(${ACADOToolkit_INCLUDE_DIRS})

add_executable(controller_node controller.cpp
                               ps3d.cpp
                               differential_drive_controller.cpp
                               stuck_detector.cpp
                               four_wheel_steer_controller.cpp
                               melman_mpc_wrapper.cpp)
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                 monstertruck_msgs_generate_messages_cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES} ${ACADOToolkit_LIBRARIES})

add_executable(mpc_node mpc.cpp acado_mpc_wrapper.cpp)
add_dependencies(mpc_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                          monstertruck_msgs_generate_messages_cpp)
target_link_libraries(mpc_node ${catkin_LIBRARIES}
                               ${ACADOToolkit_LIBRARIES}
                               ${Boost_LIBRARIES})


