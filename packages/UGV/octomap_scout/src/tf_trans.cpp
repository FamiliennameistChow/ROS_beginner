#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

// http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
// https://www.jianshu.com/p/864b9a67dc20

Eigen::Matrix4f laser_pose = Eigen::Matrix4f::Identity();
Eigen::Matrix4f odom_spose = Eigen::Matrix4f::Identity();
Eigen::Matrix4f T_odom_laser = Eigen::Matrix4f::Identity();
Eigen::Matrix4f T_laser_odom = Eigen::Matrix4f::Identity();
void laser_loca_cb(const nav_msgs::OdometryConstPtr& laser_msg_ptr){
    laser_pose(0,3) = laser_msg_ptr->pose.pose.position.x;
    laser_pose(1,3) = laser_msg_ptr->pose.pose.position.y;
    laser_pose(2,3) = laser_msg_ptr->pose.pose.position.z;
    Eigen::Quaternionf q;
    q.x() = laser_msg_ptr->pose.pose.orientation.x;
    q.y() = laser_msg_ptr->pose.pose.orientation.y;
    q.z() = laser_msg_ptr->pose.pose.orientation.z;
    q.w() = laser_msg_ptr->pose.pose.orientation.w;
    laser_pose.block<3,3>(0,0) = q.matrix();
}

void odom_loca_cb(const nav_msgs::OdometryConstPtr& odom_msg_ptr){
    odom_spose(0,3) = odom_msg_ptr->pose.pose.position.x;
    odom_spose(1,3) = odom_msg_ptr->pose.pose.position.y;
    odom_spose(2,3) = odom_msg_ptr->pose.pose.position.z;
    Eigen::Quaternionf q1;
    q1.x() = odom_msg_ptr->pose.pose.orientation.x;
    q1.y() = odom_msg_ptr->pose.pose.orientation.y;
    q1.z() = odom_msg_ptr->pose.pose.orientation.z;
    q1.w() = odom_msg_ptr->pose.pose.orientation.w;
    odom_spose.block<3,3>(0,0) = q1.matrix();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_trans");
    ros::NodeHandle private_nh("~");
    ros::Rate rate(50);

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Subscriber laser_loca_estimation_sb = private_nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 10, laser_loca_cb);
    ros::Subscriber odom_loca_estimation_sb = private_nh.subscribe<nav_msgs::Odometry>("/odom", 10, odom_loca_cb);

    while(ros::ok())
    {
        T_odom_laser = laser_pose * odom_spose.inverse();
        T_laser_odom = odom_spose * laser_pose.inverse();
        
        //  Eigen::Matrix4f to tf::Transform
        Eigen::Quaterniond eigen_quat(T_laser_odom.block<3,3>(0,0).cast<double>());
        Eigen::Vector3d eigen_trans(T_laser_odom.block<3,1>(0,3).cast<double>());
        tf::Quaternion tf_quat(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
        ROS_INFO("=== the pose of odom  to map is: xyz-> %f, %f, %f \n  Quaterniond ->  %f, %f, %f, %f",
        eigen_trans(0), eigen_trans(1), eigen_trans(2), eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());

        transform.setOrigin(tf::Vector3(eigen_trans(0), eigen_trans(1), eigen_trans(2)));
        transform.setRotation(tf_quat);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}