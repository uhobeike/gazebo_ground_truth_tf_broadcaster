cmake_minimum_required(VERSION 3.0.2)
project(gazebo_ground_truth_tf_broadcaster)

set(CMAKE_CXX_FLAGS "-std=c++14 -O3 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gazebo_ground_truth_tf_broadcaster
  src/gazebo_ground_truth_tf_broadcaster.cpp
)
target_link_libraries(gazebo_ground_truth_tf_broadcaster
  ${catkin_LIBRARIES}
)

install(TARGETS gazebo_ground_truth_tf_broadcaster
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)