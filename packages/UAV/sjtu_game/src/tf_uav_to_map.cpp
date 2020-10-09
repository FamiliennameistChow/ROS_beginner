#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

// http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
// https://www.jianshu.com/p/864b9a67dc20

std::string sensorFrameId, robotFrameId, mapFrameId;
std::string robotAttitudeTopic;
double x, y, z, R, P, Y;

geometry_msgs::PoseStamped local_position;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    local_position = *msg;
    tf::TransformBroadcaster br;
    //　初始化变换参数
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z));
    transform.setRotation(tf::Quaternion(local_position.pose.orientation.x, 
                                        local_position.pose.orientation.y,
                                        local_position.pose.orientation.z, 
                                        local_position.pose.orientation.w));
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, mapFrameId, robotFrameId));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_uav2map");
    ros::NodeHandle nh("~");
    ros::Rate rate(50);

    nh.param<std::string>("sensor_frame", sensorFrameId, std::string("camera_link"));
    nh.param<std::string>("robot_frame", robotFrameId, std::string("base_link"));
    nh.param<std::string>("map_frame", mapFrameId, std::string("map"));
    nh.param<std::string>("robot_attitude_topic", robotAttitudeTopic, std::string("/mavros/local_position/pose"));
    nh.param<double>("sensor_transform_from_robot/translation/x", x, 0.0);
    nh.param<double>("sensor_transform_from_robot/translation/y", y, 0.0);
    nh.param<double>("sensor_transform_from_robot/translation/z", z, 0.0);
    nh.param<double>("sensor_transform_from_robot/rotation/r", R, -1.57);
    nh.param<double>("sensor_transform_from_robot/rotation/p", P, 0.0);
    nh.param<double>("sensor_transform_from_robot/rotation/y", Y, -1.57);
    
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(robotAttitudeTopic, 10, local_position_cb);
    
    tf::TransformBroadcaster br2;
    tf::Transform transform2;
    while(ros::ok())
    {
        transform2.setOrigin(tf::Vector3(x, y, z));
        tf::Quaternion q;
        q.setRPY(R, P, Y);
        transform2.setRotation(q);
        br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), robotFrameId, sensorFrameId));
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}