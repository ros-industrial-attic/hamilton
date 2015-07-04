cmake_minimum_required(VERSION 2.8.3)
project(hamilton)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  moveit_msgs
  moveit_ros_planning_interface
  trajectory_msgs	
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

catkin_package()

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_demo src/main.cpp)

target_link_libraries(${PROJECT_NAME}_demo ${catkin_LIBRARIES})

