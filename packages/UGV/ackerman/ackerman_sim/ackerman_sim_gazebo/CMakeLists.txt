cmake_minimum_required(VERSION 3.0.2)
project(ackerman_sim_gazebo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    include
)

add_executable(tf_base2map src/robot_setup_tf/tf_base2map.cpp)
target_link_libraries(tf_base2map  ${catkin_LIBRARIES})

add_executable(sim_trajectory src/sim_trajectory.cpp)
target_link_libraries(sim_trajectory  ${catkin_LIBRARIES})
