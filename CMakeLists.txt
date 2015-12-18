cmake_minimum_required(VERSION 2.8.3)
project(vehicle_controller)

find_package(catkin REQUIRED COMPONENTS tf dynamic_reconfigure nav_msgs geometry_msgs cmake_modules hector_move_base_msgs monstertruck_msgs)

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
#  LIBRARIES test
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

#############
## Testing ##
#############

add_dependencies(controller_node ${PROJECT_NAME}_gencfg)
