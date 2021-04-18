cmake_minimum_required(VERSION 2.8.3)
project(sample_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_control
  roscpp
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS
  geometry_msgs
  move_control
  roscpp
  std_msgs
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${PROJECT_NAME}_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
