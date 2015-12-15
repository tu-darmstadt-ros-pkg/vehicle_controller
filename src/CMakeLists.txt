include_directories(${catkin_INCLUDE_DIRS} include)

include_directories("/home/paul/ACADOtoolkit"
                    "/home/paul/ACADOtoolkit/acado"
                    "/home/paul/ACADOtoolkit/acado/bindings")
find_library(acado_lib
             acado_toolkit_s
             "/home/paul/ACADOtoolkit/build/lib")

add_executable(controller_node controller.cpp
                               ps3d.cpp
                               differential_drive_controller.cpp
                               stuck_detector.cpp
                               four_wheel_steer_controller.cpp
                               melman_mpc_wrapper.cpp)
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} monstertruck_msgs_generate_messages_cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES} ${acado_lib})
