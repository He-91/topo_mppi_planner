cmake_minimum_required(VERSION 2.8.3)
project(ddo_path_searching)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# OpenMP for parallel MPPI sampling
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# CUDA for GPU-accelerated MPPI
find_package(CUDA QUIET)
if(CUDA_FOUND)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  # Auto-detect GPU architecture instead of hardcoding sm_86
  include(FindCUDA)
  if(NOT DEFINED CUDA_ARCH)
    set(CUDA_ARCH "native" CACHE STRING "CUDA architecture (e.g. sm_86, native)")
  endif()
  if(CUDA_ARCH STREQUAL "native")
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3")
    set(CMAKE_CUDA_ARCHITECTURES OFF)
  else()
    string(REPLACE "sm_" "" CUDA_ARCH_NUM ${CUDA_ARCH})
    set(CMAKE_CUDA_ARCHITECTURES ${CUDA_ARCH_NUM})
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3")
  endif()
  message(STATUS "CUDA ${CUDA_VERSION} found, GPU MPPI enabled (arch=${CUDA_ARCH})")
  add_definitions(-DUSE_GPU_MPPI)
else()
  message(WARNING "CUDA not found, falling back to CPU-only MPPI")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  ddo_plan_env
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ddo_path_searching
 CATKIN_DEPENDS ddo_plan_env
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

if(CUDA_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})
endif()

add_library( ddo_path_searching 
    src/mppi_planner.cpp
    src/topo_prm.cpp
    )

if(CUDA_FOUND)
    add_library( path_searching_gpu
        src/mppi_gpu_planner.cu
    )
    set_target_properties(path_searching_gpu PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_RESOLVE_DEVICE_SYMBOLS ON
    )
    target_include_directories( path_searching_gpu PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
    )
    target_link_libraries( path_searching_gpu
        ${CUDA_cudart_LIBRARY}
        ${catkin_LIBRARIES}
    )
endif()

target_link_libraries( ddo_path_searching
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )

if(CUDA_FOUND)
    target_link_libraries( ddo_path_searching
        path_searching_gpu
    )
endif()
