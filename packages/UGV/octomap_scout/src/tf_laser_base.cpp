#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

// http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
// https://www.jianshu.com/p/864b9a67dc20

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_laser_base");
    ros::NodeHandle nh("~");
    ros::Rate rate(50);
    
    tf::TransformBroadcaster br;
    tf::Transform transform;

    while(ros::ok())
    {
        transform.setOrigin(tf::Vector3(0, -0.4, 0));
        tf::Quaternion q;
        q.setRPY(0, -1.5707963, -1.5707963);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "base_link"));
        // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera_init"));
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}