cmake_minimum_required(VERSION 2.8.3)
project(ddo_plan_env)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)

# Check for CUDA to enable GPU features
find_package(CUDA QUIET)
if(CUDA_FOUND)
  add_definitions(-DUSE_GPU_MPPI)
  include_directories(${CUDA_INCLUDE_DIRS})
  message(STATUS "[ddo_plan_env] CUDA found, USE_GPU_MPPI defined")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ddo_plan_env
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( ddo_plan_env 
    src/grid_map.cpp 
    src/raycast.cpp
    )
target_link_libraries( ddo_plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )

# Link CUDA runtime if available
if(CUDA_FOUND)
  target_link_libraries( ddo_plan_env
      ${CUDA_LIBRARIES}
      )
endif()  
