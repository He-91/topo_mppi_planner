cmake_minimum_required(VERSION 2.8.3)
project(path_searching)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 🚀 Enable OpenMP for parallel MPPI sampling
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message(STATUS "OpenMP found for path_searching, MPPI parallel sampling enabled")
endif()

# 🔥 Enable CUDA for GPU-accelerated MPPI
find_package(CUDA QUIET)
if(CUDA_FOUND)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_86 -O3")
  message(STATUS "CUDA found: ${CUDA_VERSION}, GPU-accelerated MPPI enabled")
  message(STATUS "CUDA Architecture: sm_86 (RTX 5070)")
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
  plan_env
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching
 CATKIN_DEPENDS plan_env
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

add_library( path_searching 
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

target_link_libraries( path_searching
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )

if(CUDA_FOUND)
    target_link_libraries( path_searching
        path_searching_gpu
    )
endif()  

add_executable( test_topo_mppi
    src/test_topo_mppi.cpp
    )
target_link_libraries( test_topo_mppi
    path_searching
    ${catkin_LIBRARIES}
    )

add_executable( test_mppi_modules
    src/test_mppi_modules.cpp
    )
target_link_libraries( test_mppi_modules
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    )

if(CUDA_FOUND)
    add_executable( test_gpu_mppi
        src/test_gpu_mppi.cpp
        )
    target_include_directories( test_gpu_mppi PRIVATE
        ${CUDA_INCLUDE_DIRS}
        )
    target_link_libraries( test_gpu_mppi
        path_searching
        path_searching_gpu
        ${catkin_LIBRARIES}
        ${Eigen3_LIBRARIES}
        )
    
    add_executable( benchmark_gpu_mppi
        src/benchmark_gpu_mppi.cpp
        )
    target_include_directories( benchmark_gpu_mppi PRIVATE
        ${CUDA_INCLUDE_DIRS}
        )
    target_link_libraries( benchmark_gpu_mppi
        path_searching
        path_searching_gpu
        ${catkin_LIBRARIES}
        ${Eigen3_LIBRARIES}
        )
endif()
