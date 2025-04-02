cmake_minimum_required(VERSION 3.0.2)
project(my_bezier_global_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  costmap_2d
  global_planner
  pluginlib
  tf
  tf2
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_bezier_global_planner
  CATKIN_DEPENDS roscpp nav_core costmap_2d global_planner pluginlib tf tf2 tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(my_bezier_global_planner
  src/my_bezier_global_planner.cpp
)

target_link_libraries(my_bezier_global_planner
  ${catkin_LIBRARIES}
)

install(TARGETS my_bezier_global_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)