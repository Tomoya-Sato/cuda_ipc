cmake_minimum_required(VERSION 2.8.3)
project(ipc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  ipc_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(CUDA REQUIRED)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ipc
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs std_msgs std_msgs ipc_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

set(GPU_MNG
  node/gpu_manager/gpu_manager.cpp node/gpu_manager/gpu_manager_cuda.cu
)

set(GPU_VGF
  node/voxel_grid_filter/voxel_grid_filter.cpp node/voxel_grid_filter/voxel_grid_filter_cuda.cu
)

set(NDT_NODE
  node/ndt_node/ndt_node.cpp node/ndt_node/ndt_cuda.cu
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)


cuda_add_executable(gpu_manager ${GPU_MNG})
target_link_libraries(gpu_manager ${catkin_LIBRARIES} ${CUDA_LIBRARIES})

cuda_add_executable(ndt_node ${NDT_NODE})
target_link_libraries(ndt_node ${catkin_LIBRARIES} ${CUDA_LIBRARIES})

## cuda_add_executable(ipc_euc ${EUC_SRC})
## target_link_libraries(ipc_euc ${catkin_LIBRARIES} ${CUDA_LIBRARIES})

cuda_add_executable(voxelgridfilter ${GPU_VGF})
target_link_libraries(voxelgridfilter ${catkin_LIBRARIES} ${CUDA_LIBRARIES})
