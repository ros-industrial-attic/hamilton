cmake_minimum_required(VERSION 2.8.3)
project(hamilton)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  trajectory_msgs	
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_trial src/trial.cpp)

 target_link_libraries(${PROJECT_NAME}_trial
   ${catkin_LIBRARIES}
 )

set_target_properties(${PROJECT_NAME}_trial
  PROPERTIES OUTPUT_NAME trial
  PREFIX "")


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_trial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)