cmake_minimum_required(VERSION 2.8.3)
project(topo_mppi_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 🚀 Enable GPU MPPI acceleration
find_package(CUDA QUIET)
if(CUDA_FOUND)
  add_definitions(-DUSE_GPU_MPPI)
  message(STATUS "CUDA found, enabling USE_GPU_MPPI macro in topo_mppi_planner")
else()
  message(WARNING "CUDA not found, GPU MPPI disabled in topo_mppi_planner")
endif()

# 🚀 Enable OpenMP for parallel optimization
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
  plan_env
  path_searching
  bspline_opt
  traj_utils
  message_generation
  cv_bridge
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  DataDisp.msg
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
 LIBRARIES topo_mppi_planner
 CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# 🚀 Add CUDA include directories if available
if(CUDA_FOUND)
  include_directories(${CUDA_INCLUDE_DIRS})
endif()


add_executable(topo_mppi_planner_node
  src/topo_mppi_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(topo_mppi_planner_node 
  ${catkin_LIBRARIES}
  )
# 🚀 Link CUDA libraries if available
if(CUDA_FOUND)
  target_link_libraries(topo_mppi_planner_node ${CUDA_LIBRARIES})
endif()
add_dependencies(topo_mppi_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})


