cmake_minimum_required(VERSION 2.8.3)
project(vehicle_controller)

find_package(catkin REQUIRED COMPONENTS tf actionlib dynamic_reconfigure nav_msgs geometry_msgs cmake_modules move_base_lite_msgs monstertruck_msgs)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS} include)
add_definitions(${EIGEN_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################

generate_dynamic_reconfigure_options(
  cfg/PdParams.cfg
  cfg/PdParamsArgo.cfg
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
add_subdirectory(src)

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

#############
## Testing ##
#############

add_dependencies(controller_node ${PROJECT_NAME}_gencfg)
