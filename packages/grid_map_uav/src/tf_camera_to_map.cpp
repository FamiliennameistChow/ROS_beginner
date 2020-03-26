#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

// http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
// https://www.jianshu.com/p/864b9a67dc20

geometry_msgs::PoseStamped local_position;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    local_position = *msg;
    static tf::TransformBroadcaster br;
    //　初始化变换参数
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z));
    transform.setRotation(tf::Quaternion(local_position.pose.orientation.x, 
                                        local_position.pose.orientation.y,
                                        local_position.pose.orientation.z, 
                                        local_position.pose.orientation.w));
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "base_link"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_camera2map");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_position_cb);
    
    tf::TransformBroadcaster br2;
    tf::Transform transform2;
    while(ros::ok())
    {
        transform2.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setRPY(0.0, 3.14, 1.57);
        transform2.setRotation(q);
        br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "camera_link"));
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}