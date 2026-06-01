cmake_minimum_required(VERSION 3.0.2)
project(gt_motion_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  gazebo_msgs
  visualization_msgs
  tf
)

find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs gazebo_msgs visualization_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(gt_planner_node
  src/gt_planner_node.cpp
)

target_link_libraries(gt_planner_node
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
)
