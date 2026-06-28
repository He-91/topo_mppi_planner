cmake_minimum_required(VERSION 3.0.2)
project(ddo_benchmark)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf
  quadrotor_msgs
  pcl_conversions
  ddo_planner
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs sensor_msgs visualization_msgs tf quadrotor_msgs pcl_conversions ddo_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Benchmark metrics collector node
add_executable(benchmark_node src/benchmark_node.cpp)
add_dependencies(benchmark_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(benchmark_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Goal publisher node (sends waypoints automatically for benchmark)
add_executable(goal_publisher src/goal_publisher.cpp)
add_dependencies(goal_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_publisher ${catkin_LIBRARIES})
