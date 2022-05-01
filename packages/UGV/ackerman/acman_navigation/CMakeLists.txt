cmake_minimum_required(VERSION 3.0.2)
project(acman_navigation)

add_compile_options(-std=c++14)

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  dynamic_reconfigure
  nodelet
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    costmap_2d
    message_generation
    diagnostic_msgs
    ${PACKAGE_DEPENDENCIES}
)

find_package(octomap REQUIRED)
# find_package(Eigen REQUIRED) 
find_package(PCL REQUIRED )
find_package(Boost REQUIRED COMPONENTS system)
add_definitions(-DOCTOMAP_NODEBUGOUT)

# generate_dynamic_reconfigure_options(config/OctomapServer.cfg)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
    FILES
    ElevationGrid.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    nav_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime costmap_2d diagnostic_msgs
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    INCLUDE_DIRS include
    DEPENDS OCTOMAP PCL EIGEN
)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

link_directories(
	include
    ${PCL_LIBRARY_DIRS}
)

add_executable(trajectory_server src/navigation/trajectory_server.cpp)
target_link_libraries(trajectory_server  ${catkin_LIBRARIES} )

add_executable(trajectory_test src/navigation/trajectory_test.cpp)
target_link_libraries(trajectory_test  ${catkin_LIBRARIES} )