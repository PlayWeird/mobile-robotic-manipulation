cmake_minimum_required(VERSION 3.0.2)
project(touch_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  geometry_msgs
  tf2_ros
  read_targets
)

find_package(TinyXML REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES touch_planner
  CATKIN_DEPENDS cmake_modules roscpp geometry_msgs tf2_ros read_targets
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${TinyXML_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS ${PROJECT_NAME}_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
