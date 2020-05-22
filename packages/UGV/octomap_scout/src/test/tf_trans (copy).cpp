#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

// http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
// https://www.jianshu.com/p/864b9a67dc20

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_trans");
    ros::NodeHandle private_nh("~");
    ros::Rate rate(50);

    tf::TransformListener listener;
    
    tf::TransformBroadcaster br;
    // tf::Transform transform;
    tf::StampedTransform transform; 

    // 等待获取监听信息camera_init和aft_mapped
    listener.waitForTransform("camera_init", "aft_mapped", ros::Time(0), ros::Duration(3.0));


    while(ros::ok())
    {
        try{
            listener.lookupTransform("camera_init", "aft_mapped", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            // ros::Duration(1.0).sleep();
            continue;
        }
        ROS_INFO("listen success!! "); 
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_init", "base_link"));
        ros::spinOnce();
        // rate.sleep();
    }
    return 0;
}