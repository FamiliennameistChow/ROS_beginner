 /**************************************************************************
 * process_pc_map.cpp
 * 
 * Author： Born Chow
 * Date: 2021.08.16
 * 
 * 说明: 处理从激光slam获取的点云地图
 * 【订阅】
 * 【发布】 1.处理后的激光点云地图 /laser_cloud_map_preprocess 
 ***************************************************************************/

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
typedef pcl::PointXYZI  PointType;

class ProcessPCMap
{
private:
    ros::NodeHandle nh;

    ros::Subscriber laser_map_sub_;
    ros::Subscriber laser_odom_sub_;

    ros::Publisher laser_local_map_pub_;

    PointType cloudPose3D;
    PointType local_map_area;
    int local_map_count;
    int global_map_load_thre;  // 每隔global_map_load_thre次,加载一次全局地图
    pcl::PassThrough<PointType> pass_x_;
    pcl::PassThrough<PointType> pass_y_;
    pcl::PassThrough<PointType> pass_z_;

    string pointcloud_topic_;
    string laser_odom_topic_;
    string slam_type_;
    pcl::PointCloud<PointType>::Ptr gloabl_map_ptr_;
    

private:
    void laser_cloud_map_cb(const sensor_msgs::PointCloud2ConstPtr& msg);

    void laser_odom_cb(const nav_msgs::Odometry::ConstPtr &laser_odom_msg);


public:
    ProcessPCMap(ros::NodeHandle &nh, ros::NodeHandle& nh_);
    ~ProcessPCMap();
};

ProcessPCMap::ProcessPCMap(ros::NodeHandle &nh, ros::NodeHandle& nh_) :
nh(nh),
gloabl_map_ptr_(new pcl::PointCloud<PointType>())
{

    //加载全局变量
    nh.param<string>("/slam_algorithm", slam_type_, "lio-sam");
    nh.param<string>("/pc_topic", pointcloud_topic_, "/lio_sam/mapping/cloud_registered"); //for aloam  /laser_cloud_map
    nh.param<string>("/laser_odom_potic", laser_odom_topic_, "/lio_sam/mapping/odometry");  // for aloam  /integrated_to_init

    laser_map_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic_, 10, &ProcessPCMap::laser_cloud_map_cb, this);
    laser_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(laser_odom_topic_, 10, &ProcessPCMap::laser_odom_cb, this);

    laser_local_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map_preprocess", 10);

    // init param
    pass_x_.setFilterFieldName("x");
    pass_y_.setFilterFieldName("y");
    pass_z_.setFilterFieldName("z");
    local_map_count = 0;

    //加载私有变量
    nh_.param<float>("local_map_x", local_map_area.x, 20.0);
    nh_.param<float>("local_map_y", local_map_area.y, 20.0);
    nh_.param<float>("local_map_z", local_map_area.z, 5.0);
    nh_.param<int>("global_map_load_thre", global_map_load_thre, 20);

    cout << "=========================================" << endl;
    cout << "param confirm..." << endl;
    cout << "slam Type      |  " << slam_type_ << endl;
    cout << "pointcloud map |  " << pointcloud_topic_ << endl;
    cout << "odom           |  " << laser_odom_topic_ << endl;
    cout << "========================================" << endl;

}

ProcessPCMap::~ProcessPCMap()
{
}

void ProcessPCMap::laser_odom_cb(const nav_msgs::Odometry::ConstPtr &laser_odom_msg){
    cloudPose3D.x = laser_odom_msg->pose.pose.position.x;
    cloudPose3D.y = laser_odom_msg->pose.pose.position.y;
    cloudPose3D.z = laser_odom_msg->pose.pose.position.z;
    cloudPose3D.intensity = laser_odom_msg->header.stamp.toSec();
}

void ProcessPCMap::laser_cloud_map_cb(const sensor_msgs::PointCloud2ConstPtr& msg){

    if(slam_type_ == "lio-sam"){
        pcl::PointCloud<PointType>::Ptr reg_pc_ptr(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*msg, *reg_pc_ptr);
        *gloabl_map_ptr_ += *reg_pc_ptr;   
    }
    else if (slam_type_ == "aloam")
    {
        pcl::fromROSMsg(*msg, *gloabl_map_ptr_);
        
    }
    else
    {
        cout << "!!!!no such laser slam " << slam_type_ << endl;
        cout << "now support [1]aloam and [2]lio-sam " << endl;
    }
    

    

    if (local_map_count % global_map_load_thre != 0) //每隔global_map_load_thre次,不修剪,即是加载一次全局地图
    {
        //设置裁剪区域
        pass_x_.setFilterLimits(cloudPose3D.x - local_map_area.x/2, cloudPose3D.x + local_map_area.x/2);
        pass_y_.setFilterLimits(cloudPose3D.y - local_map_area.y/2, cloudPose3D.y + local_map_area.y/2);
        pass_z_.setFilterLimits(cloudPose3D.z - local_map_area.z/2, cloudPose3D.z + local_map_area.z/2);

        pass_x_.setInputCloud((*gloabl_map_ptr_).makeShared());
        pass_x_.filter(*gloabl_map_ptr_);
        pass_y_.setInputCloud((*gloabl_map_ptr_).makeShared());
        pass_y_.filter(*gloabl_map_ptr_);
        pass_z_.setInputCloud((*gloabl_map_ptr_).makeShared());
        pass_z_.filter(*gloabl_map_ptr_);
    }
    
    local_map_count++;

    sensor_msgs::PointCloud2 tempMsgCloud;
    pcl::toROSMsg(*gloabl_map_ptr_, tempMsgCloud);
    tempMsgCloud.header.stamp = ros::Time();
    tempMsgCloud.header.frame_id = msg->header.frame_id;
    laser_local_map_pub_.publish(tempMsgCloud);
}


// ==========================class end =========================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "process_pc_map");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    ProcessPCMap processPC(nh, nh_);

    ros::spin();
    return 0;

}