cmake_minimum_required(VERSION 2.8.3)
project(ddo_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# GPU MPPI acceleration
find_package(CUDA QUIET)
if(CUDA_FOUND)
  add_definitions(-DUSE_GPU_MPPI)
  message(STATUS "CUDA found, GPU MPPI enabled in ddo_planner")
else()
  message(WARNING "CUDA not found, GPU MPPI disabled in ddo_planner")
endif()

# OpenMP for parallel optimization
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message(STATUS "OpenMP found, parallel optimization enabled")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  ddo_plan_env
  ddo_path_searching
  ddo_bspline_opt
  ddo_traj_utils
  message_generation
  cv_bridge
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  DataDisp.msg
  DynamicObstacle.msg
  DynamicObstacles.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ddo_plan_env ddo_path_searching ddo_bspline_opt ddo_traj_utils message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Add CUDA include directories if available
if(CUDA_FOUND)
  include_directories(${CUDA_INCLUDE_DIRS})
endif()


add_executable(ddo_planner_node
  src/ddo_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(ddo_planner_node 
  ${catkin_LIBRARIES}
  )
# Link CUDA libraries if available
if(CUDA_FOUND)
  target_link_libraries(ddo_planner_node ${CUDA_LIBRARIES})
endif()
add_dependencies(ddo_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})


