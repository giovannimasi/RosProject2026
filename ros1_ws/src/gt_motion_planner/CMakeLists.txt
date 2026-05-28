cmake_minimum_required(VERSION 3.0.2)
project(gt_motion_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  gazebo_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gt_planner_node
  src/gt_planner_node.cpp
)

target_link_libraries(gt_planner_node
  ${catkin_LIBRARIES}
)
