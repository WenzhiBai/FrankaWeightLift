cmake_minimum_required(VERSION 3.4)
project(franka_weight_lift)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  franka_hw
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
)

find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)

# merge Franka + INCLUDE_DIRS in topological order
list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  cfg/weight_lift_param.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    weight_lift_controller
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
  DEPENDS
    Franka
)

# weight_lift_controller: the franka_hw controller plugin
file(GLOB_RECURSE WEIGHT_LIFT_CONTROLLER_SRC src/*.cpp)
add_library(weight_lift_controller ${WEIGHT_LIFT_CONTROLLER_SRC})

add_dependencies(weight_lift_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(weight_lift_controller PUBLIC
  ${catkin_LIBRARIES}
  ${Franka_LIBRARIES}
)

target_include_directories(weight_lift_controller SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(weight_lift_controller PUBLIC
  include
)

## Installation
install(TARGETS
  weight_lift_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
