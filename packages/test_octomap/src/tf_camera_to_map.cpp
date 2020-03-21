# include <ros/ros.h>
# include <tf/transform_broadcaster.h>
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
    transform.setOrigin(tf::Vector3(local_position.pose.position.y, local_position.pose.position.x, local_position.pose.position.z));
    transform.setRotation(tf::Quaternion(local_position.pose.orientation.w, 
                                        local_position.pose.orientation.x,
                                        local_position.pose.orientation.y, 
                                        local_position.pose.orientation.z));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera_link"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_camera2map");
    ros::NodeHandle nh;

    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_position_cb);

    while(nh.ok()){

        ros::spin();
    }
    return 0;
}