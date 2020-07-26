# include <ros/ros.h>
# include <tf/transform_listener.h>
# include <geometry_msgs/PointStamped.h>
# include <sensor_msgs/PointCloud2.h>
# include <iostream>
# include <pcl_ros/point_cloud.h>
# include <pcl_ros/transforms.h>
# include <pcl_ros/impl/transforms.hpp>
# include <pcl_conversions/pcl_conversions.h>

# include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/keypoints/uniform_sampling.h>
# include <pcl/filters/passthrough.h>

using namespace std;

typedef pcl::PointXYZRGB PCLPoint;

namespace gridmap_scout {

class ProcessKinectPC {
public:
    ProcessKinectPC(ros::NodeHandle &nodeHandle, double m_pointcloudMinZ, double m_pointcloudMaxZ);

    virtual ~ProcessKinectPC();

    void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);

private:
    ros::NodeHandle &nodeHandle_;

    ros::Subscriber pointcloud_sub_;

    ros::Publisher pointcloud_pub_;

    //kinect的Ｙ轴为ｍａｐ坐标系的Ｚ轴。且正Ｙ相当于负Ｚ
    double m_pointcloudMinZ_;
    
    double m_pointcloudMaxZ_;

    pcl::PassThrough<PCLPoint> pass_z_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_data_sub_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_uniformsampled_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthroughed_;

    sensor_msgs::PointCloud2 pointcloud_, pointcloud_data_pub_;

};



ProcessKinectPC::ProcessKinectPC(ros::NodeHandle &nodeHandle, double m_pointcloudMinZ, double m_pointcloudMaxZ)
    : nodeHandle_(nodeHandle), m_pointcloudMinZ_(m_pointcloudMinZ), m_pointcloudMaxZ_(m_pointcloudMaxZ),
        pointcloud_data_sub_(new pcl::PointCloud<pcl::PointXYZRGB>),
        cloud_uniformsampled_(new pcl::PointCloud<pcl::PointXYZRGB>),
        cloud_filtered_(new pcl::PointCloud<pcl::PointXYZRGB>),
        cloud_passthroughed_(new pcl::PointCloud<pcl::PointXYZRGB>){

        pointcloud_sub_ = nodeHandle_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 10, &ProcessKinectPC::pointcloud_cb, this);
        pointcloud_pub_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/robot_base/points", 10, true);

        pass_z_.setFilterFieldName("z");
        pass_z_.setFilterLimits(m_pointcloudMinZ_, m_pointcloudMaxZ_);
        pass_z_.setFilterLimitsNegative(true);
    }

ProcessKinectPC::~ProcessKinectPC() {}


void ProcessKinectPC::pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg){
    // tf::StampedTransform transform;
    tf::TransformListener listener_;
    try {
        listener_.waitForTransform("base_link", "camera_depth_optical_frame", ros::Time(0),
                                ros::Duration(3.0));
        // listener_.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0),
        //                         transform);

    } catch (tf::TransformException &ex) {
        ROS_ERROR("frame transform error: %s", ex.what());
        return;
    }

    pcl_ros::transformPointCloud("base_link", *msg, pointcloud_, listener_);

    pcl::fromROSMsg(pointcloud_, *pointcloud_data_sub_);
    std::cout << "Original cloud: " << std::endl;
    std::cout << *pointcloud_data_sub_ << std::endl;

    // 均匀采样，取一定半径 r 的球体内的点保留质心。
    pcl::UniformSampling<pcl::PointXYZRGB> filter;
    filter.setInputCloud(pointcloud_data_sub_);
    filter.setRadiusSearch(0.03);
    // We need an additional object to store the indices of surviving points.
    pcl::PointCloud<int> keypointIndices;
    filter.compute(keypointIndices);
    pcl::copyPointCloud(*pointcloud_data_sub_, keypointIndices.points, *cloud_uniformsampled_);
    std::cerr << "Cloud after uniformSampling: " << std::endl;
    std::cerr << *cloud_uniformsampled_ << std::endl;


    // 滤波移除离群点
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_uniformsampled_);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered_);
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered_ << std::endl;

    pass_z_.setInputCloud(cloud_filtered_);
    pass_z_.filter(*cloud_passthroughed_);

    pcl::toROSMsg(*cloud_passthroughed_, pointcloud_data_pub_);
    pointcloud_data_pub_.header.stamp = msg -> header.stamp;
    pointcloud_data_pub_.header.frame_id = "base_link";
    pointcloud_pub_.publish(pointcloud_data_pub_);


    ROS_INFO("have processing Kinect's pointcloud.");
}

}