cmake_minimum_required(VERSION 3.0.2)
project(vehicle_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")

find_package(catkin REQUIRED COMPONENTS tf actionlib dynamic_reconfigure nav_msgs geometry_msgs cmake_modules move_base_lite_msgs monstertruck_msgs eigen_conversions)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################

generate_dynamic_reconfigure_options(
  cfg/PdParams.cfg
  cfg/ControllerType.cfg
  cfg/DafControllerParams.cfg
  cfg/CarrotControllerParams.cfg
  cfg/PurePursuitControllerParams.cfg
  cfg/LqrControllerParams.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vehicle_controller_lib
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


###########
## Build ##
###########

#add_subdirectory(src)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(vehicle_controller_lib src/ps3d.cpp src/differential_drive_controller.cpp src/stuck_detector.cpp src/four_wheel_steer_controller.cpp src/daf_controller.cpp
  src/carrot_controller.cpp src/ackermann_pure_pursuit_controller src/differential_pure_pursuit_controller src/ekf.cpp src/lqr_controller.cpp src/controller.cpp)
add_dependencies(vehicle_controller_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(vehicle_controller_lib ${catkin_LIBRARIES})

add_executable(controller_node src/controller_node.cpp)
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} vehicle_controller_lib ${PROJECT_NAME}_gencfg)
target_link_libraries(controller_node vehicle_controller_lib ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS vehicle_controller_lib controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY 
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)