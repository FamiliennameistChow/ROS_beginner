#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

// http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
// https://www.jianshu.com/p/864b9a67dc20

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_trans");
    ros::NodeHandle private_nh("~");
    ros::Rate rate(50);
    
    tf::TransformBroadcaster br;
    tf::Transform transform;

    float R,P,Y,x,y,z;
    std::string parent_frame, child_frame;
    private_nh.param<float>("x", x, 0.0);
    private_nh.param<float>("y", y, 0.0);
    private_nh.param<float>("z", z, 0.0);
    private_nh.param<float>("R", R, 0.0);
    private_nh.param<float>("P", P, 0.0);
    private_nh.param<float>("Y", Y, 0.0);
    private_nh.param<std::string>("parent_frame", parent_frame, "base_link");
    private_nh.param<std::string>("child_frame", child_frame, "camera");
    ROS_INFO("=== the pose of %s  to  %s  is: xyz-> %f, %f, %f   RPY->  %f, %f, %f ", parent_frame.c_str(), child_frame.c_str(), x, y, z, R, P, Y);

    while(ros::ok())
    {
        // transform.setOrigin(tf::Vector3(0, -0.4, 0));
        // tf::Quaternion q;
        // q.setRPY(0, -1.5707963, -1.5707963);

        transform.setOrigin(tf::Vector3(x, y, z));
        tf::Quaternion q;
        q.setRPY(R, P, Y);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}