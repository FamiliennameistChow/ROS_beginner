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
# include <pcl/filters/passthrough.h>

using namespace std;

typedef pcl::PointXYZ PCLPoint;


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

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data_sub_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_data_sub_filtered_;

    sensor_msgs::PointCloud2 pointcloud_, pointcloud_data_pub_;

};



ProcessKinectPC::ProcessKinectPC(ros::NodeHandle &nodeHandle, double m_pointcloudMinZ, double m_pointcloudMaxZ)
    : nodeHandle_(nodeHandle), m_pointcloudMinZ_(m_pointcloudMinZ), m_pointcloudMaxZ_(m_pointcloudMaxZ),
    pointcloud_data_sub_(new pcl::PointCloud<pcl::PointXYZ>), pointcloud_data_sub_filtered_(new pcl::PointCloud<pcl::PointXYZ>) {
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

    pass_z_.setInputCloud(pointcloud_data_sub_);
    pass_z_.filter(*pointcloud_data_sub_filtered_);

    pcl::toROSMsg(*pointcloud_data_sub_filtered_, pointcloud_data_pub_);
    pointcloud_data_pub_.header.stamp = msg -> header.stamp;
    pointcloud_data_pub_.header.frame_id = "base_link";
    pointcloud_pub_.publish(pointcloud_data_pub_);


    ROS_INFO("have processing Kinect's pointcloud.");
}
