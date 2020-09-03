 /*
 * tf_trans_init.cpp
 * 
 * Author： Born Chow
 * Date: 2020.07.21
 * 
 * 说明: 获取ndt的定位信息，并发布camera_init to map的tf转换
 *
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

class TfTransInit
{
private:
    ros::NodeHandle n_;
    tf::TransformBroadcaster br_;
    tf::Transform transform_;
    tf::Transform transform_last_;
    geometry_msgs::PoseStamped ndt_pose_;

    ros::Subscriber ndt_pose_sub_;
    ros::Subscriber initial_pose_sub_;

    bool loca_init_;
    int loca_cnt_;

private:
    void callback_ndt_loca(const geometry_msgs::PoseStampedConstPtr &ndt_msg_ptr);
    void callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_conv_msg_ptr);

public:
    TfTransInit(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~TfTransInit();
};

TfTransInit::TfTransInit(ros::NodeHandle& nh, ros::NodeHandle& private_nh) :
n_(nh),
loca_init_(false)
{
    ndt_pose_sub_ = n_.subscribe<geometry_msgs::PoseStamped>("ndt_pose", 10, &TfTransInit::callback_ndt_loca, this);
    initial_pose_sub_ = n_.subscribe("initialpose", 100, &TfTransInit::callback_init_pose, this);
}


TfTransInit::~TfTransInit()
{
}

void TfTransInit::callback_ndt_loca(const geometry_msgs::PoseStampedConstPtr &ndt_msg_ptr){
    if (loca_init_ && loca_cnt_< 100)
    {
        ROS_INFO("---------------locating--------------------------  %d", loca_cnt_);
        loca_cnt_ ++;
        ndt_pose_ = *ndt_msg_ptr;
        transform_.setOrigin(tf::Vector3(ndt_pose_.pose.position.x, ndt_pose_.pose.position.y, ndt_pose_.pose.position.z));
        transform_.setRotation(tf::Quaternion(ndt_pose_.pose.orientation.x, 
                                            ndt_pose_.pose.orientation.y,
                                            ndt_pose_.pose.orientation.z, 
                                            ndt_pose_.pose.orientation.w));
        br_.sendTransform(tf::StampedTransform(transform_, ndt_msg_ptr->header.stamp, "map", "camera_init"));
        transform_last_ = transform_;
    }else
    {
        ROS_INFO("---------------LOCA FINISHED--------------------------------");
        br_.sendTransform(tf::StampedTransform(transform_last_, ndt_msg_ptr->header.stamp, "map", "camera_init"));
    }
    
    ROS_INFO("loca_cnt  %d", loca_cnt_);
}

void TfTransInit::callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & initial_pose_msg_ptr){
    if (loca_cnt_ != 100)
    {
        loca_init_ = true;
        loca_cnt_ = 0;
    }
}

// class end

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_trans_init");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    
    TfTransInit tf_init(nh, private_nh);

    ros::spin();

    return 0;
}