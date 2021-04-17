cmake_minimum_required(VERSION 3.0.2)
project(move_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    move_base_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    roscpp
    std_msgs
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${PROJECT_NAME}_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
