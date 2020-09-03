 /**************************************************************************
 * crop_global_aap.cpp
 * 
 * Author： Born Chow
 * Date: 2020.08.05
 * 
 * 说明: 以机器人为中心裁剪aloam发布的全局地图，将返回的局部地图输出八叉树以减少八叉树负担
 * 【订阅】
 ***************************************************************************/

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZI  PointType;

class CropGlobalMap
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
    

private:
    void laser_cloud_map_cb(const sensor_msgs::PointCloud2ConstPtr& msg);

    void laser_odom_cb(const nav_msgs::Odometry::ConstPtr &laser_odom_msg);


public:
    CropGlobalMap(ros::NodeHandle &nh, ros::NodeHandle& nh_);
    ~CropGlobalMap();
};

CropGlobalMap::CropGlobalMap(ros::NodeHandle &nh, ros::NodeHandle& nh_) :
nh(nh)
{
    laser_map_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_map", 10, &CropGlobalMap::laser_cloud_map_cb, this);
    laser_odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 10, &CropGlobalMap::laser_odom_cb, this);

    laser_local_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("laser_cloud_map_local", 10);

    // init param
    pass_x_.setFilterFieldName("x");
    pass_y_.setFilterFieldName("y");
    pass_z_.setFilterFieldName("z");
    local_map_count = 0;

    nh_.param<float>("local_map_x", local_map_area.x, 20.0);
    nh_.param<float>("local_map_y", local_map_area.y, 20.0);
    nh_.param<float>("local_map_z", local_map_area.z, 5.0);
    nh_.param<int>("global_map_load_thre", global_map_load_thre, 20);

}

CropGlobalMap::~CropGlobalMap()
{
}

void CropGlobalMap::laser_odom_cb(const nav_msgs::Odometry::ConstPtr &laser_odom_msg){
    cloudPose3D.x = laser_odom_msg->pose.pose.position.x;
    cloudPose3D.y = laser_odom_msg->pose.pose.position.y;
    cloudPose3D.z = laser_odom_msg->pose.pose.position.z;
    cloudPose3D.intensity = laser_odom_msg->header.stamp.toSec();
}

void CropGlobalMap::laser_cloud_map_cb(const sensor_msgs::PointCloud2ConstPtr& msg){
    // if(cloudPose3D.intensity - msg->header.stamp.toSec() < 0.05){
        pcl::PointCloud<PointType>::Ptr gloabl_map_ptr(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*msg, *gloabl_map_ptr);

        local_map_count++;

        if (local_map_count % global_map_load_thre != 0) //每隔global_map_load_thre次,不修剪,即是加载一次全局地图
        {
            //设置裁剪区域
            pass_x_.setFilterLimits(cloudPose3D.x - local_map_area.x/2, cloudPose3D.x + local_map_area.x/2);
            pass_y_.setFilterLimits(cloudPose3D.y - local_map_area.y/2, cloudPose3D.y + local_map_area.y/2);
            pass_z_.setFilterLimits(cloudPose3D.z - local_map_area.z/2, cloudPose3D.z + local_map_area.z/2);

            pass_x_.setInputCloud((*gloabl_map_ptr).makeShared());
            pass_x_.filter(*gloabl_map_ptr);
            pass_y_.setInputCloud((*gloabl_map_ptr).makeShared());
            pass_y_.filter(*gloabl_map_ptr);
            pass_z_.setInputCloud((*gloabl_map_ptr).makeShared());
            pass_z_.filter(*gloabl_map_ptr);
        }
        

        sensor_msgs::PointCloud2 tempMsgCloud;
        pcl::toROSMsg(*gloabl_map_ptr, tempMsgCloud);
        tempMsgCloud.header.stamp = ros::Time();
        tempMsgCloud.header.frame_id = msg->header.frame_id;
        laser_local_map_pub_.publish(tempMsgCloud);

    // }
    
}


// ==========================class end =========================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crop_global_map");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    CropGlobalMap cropmap(nh, nh_);

    ros::spin();
    return 0;

}