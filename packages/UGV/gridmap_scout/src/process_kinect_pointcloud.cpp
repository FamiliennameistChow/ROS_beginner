#include <ros/ros.h>
#include "process_kinect_pc.hpp"

int main(int argc, char ** argv){
    ros::init(argc, argv, "process_kinect_pointcloud");

    ros::NodeHandle nh;
    ros::Rate rate(30.0);

    ProcessKinectPC processKinectPC(nh, -0.8, 0.7);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}