cmake_minimum_required(VERSION 2.8.3)
project(grid_map_uav)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC_FILES
  src/pointcloud_to_gridmap/GridMapPclConverter.cpp
  src/pointcloud_to_gridmap/GridMapPclLoader.cpp
  src/pointcloud_to_gridmap/helpers.cpp
  src/pointcloud_to_gridmap/PclLoaderParameters.cpp
  src/pointcloud_to_gridmap/PointcloudProcessor.cpp
)

set(CATKIN_PACKAGE_DEPENDENCIES  
  grid_map_core
  grid_map_msgs
  grid_map_ros
  pcl_ros
  roscpp
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  geometry_msgs
  sensor_msgs
  cv_bridge
  octomap_msgs
  filters
)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

## Find catkin macros and libraries
find_package(catkin REQUIRED 
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(octomap REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(OpenMP QUIET)
if (OpenMP_FOUND)
  add_compile_options("${OpenMP_CXX_FLAGS}")
  add_definitions(-DGRID_MAP_PCL_OPENMP_FOUND=${OpenMP_FOUND})
endif()
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
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    yaml-cpp
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS} 
  ${OpenMP_CXX_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
  yaml-cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

## Declare a cpp executable
add_executable(
  depth_to_gridmap
  src/depth_to_gridmap.cpp
)

add_executable(pointcloud_to_gridmap
  src/pointcloud_to_gridmap.cpp
)

add_executable(tf_camera2map 
  src/tf_camera_to_map.cpp)

# add_executable(
#   simple_demo
#   src/simple_demo_node.cpp
# )

# add_executable(
#   tutorial_demo
#   src/tutorial_demo_node.cpp
# )

# add_executable(
#   iterators_demo
#   src/iterators_demo_node.cpp
#   src/IteratorsDemo.cpp
# )

# add_executable(
#   image_to_gridmap_demo
#   src/image_to_gridmap_demo_node.cpp
#   src/ImageToGridmapDemo.cpp
# )

# add_executable(
#   octomap_to_gridmap_demo
#   src/octomap_to_gridmap_demo_node.cpp
#   src/OctomapToGridmapDemo.cpp
# )

# add_executable(
#   move_demo
#   src/move_demo_node.cpp
# )

# add_executable(
#   iterator_benchmark
#   src/iterator_benchmark.cpp
# )

# add_executable(
#   opencv_demo
#   src/opencv_demo_node.cpp
# )

# add_executable(
#   resolution_change_demo
#   src/resolution_change_demo_node.cpp
# )

# add_executable(
#   filters_demo
#   src/filters_demo_node.cpp
#   src/FiltersDemo.cpp
# )

## Specify libraries to link a library or executable target against
target_link_libraries(
  depth_to_gridmap
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(pointcloud_to_gridmap
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

target_link_libraries(tf_camera2map 
  ${catkin_LIBRARIES})
# target_link_libraries(
#   simple_demo
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(
#   tutorial_demo
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(
#   iterators_demo
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(
#   image_to_gridmap_demo
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(
#   octomap_to_gridmap_demo
#   ${catkin_LIBRARIES}
#   ${OCTOMAP_LIBRARIES}
# )

# target_link_libraries(
#   move_demo
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(
#   iterator_benchmark
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(
#   opencv_demo
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

# target_link_libraries(
#   resolution_change_demo
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(
#   filters_demo
#   ${catkin_LIBRARIES}
# )
