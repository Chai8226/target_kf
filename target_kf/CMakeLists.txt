
cmake_minimum_required(VERSION 2.8.3)
project(target_kf)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  object_detection_msgs
  predict_trajectory_msgs
  nav_msgs
  message_filters
  Eigen3 REQUIRED
  mapping
)


catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS object_detection_msgs predict_trajectory_msgs mapping
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(target_kf 
  src/target_data.cpp
)
target_link_libraries(target_kf 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)

add_executable(target_sim_node
 src/target_sim_node.cpp
)
target_link_libraries(target_sim_node
   ${catkin_LIBRARIES}
)
add_dependencies(target_sim_node
  ${catkin_EXPORTED_TARGETS}
)