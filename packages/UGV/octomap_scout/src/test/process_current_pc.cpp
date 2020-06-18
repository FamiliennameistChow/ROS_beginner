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

sensor_msgs::PointCloud2 pointcloud_data_pub;
// sensor_msgs::PointCloud2 pointcloud_data_sub;
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sub (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> pc;
double m_pointcloudMinX=-30.0;
double m_pointcloudMaxX=30.0;
double m_pointcloudMinY=-30.0;
double m_pointcloudMaxY=30.0;
double m_pointcloudMinZ=-30.0;
double m_pointcloudMaxZ=30.0;
float timepc;

void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg){

    // pcl::fromROSMsg (*msg, *cloud_sub);//cloud is the output
    // pointcloud_data_sub = *msg;
    pcl::fromROSMsg(*msg, pc);
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "tf_listner");

    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    // tf::TransformListener tf_listener;

    ros::Rate rate(10.0);
    std::string pointcloud_trans_topic, pointcloud_in_topic;
    private_nh.param<std::string>("pointcloud_trans_topic", pointcloud_trans_topic, std::string("/velodyne_cloud_registered_self"));
    private_nh.param<std::string>("pointcloud_in_topic", pointcloud_in_topic, std::string("/velodyne_cloud_registered"));


    ros::Subscriber pointcloud_sub = n.subscribe<sensor_msgs::PointCloud2>(pointcloud_in_topic, 100, pointcloud_cb);
    ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>(pointcloud_trans_topic, 100);

    // set up filter for height range, also removes NANs:
    pcl::PassThrough<PCLPoint> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
    pcl::PassThrough<PCLPoint> pass_y;
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
    pcl::PassThrough<PCLPoint> pass_z;
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

    while(ros::ok())
    {   
        
        pass_x.setInputCloud(pc.makeShared());
        pass_x.filter(pc);
        pass_y.setInputCloud(pc.makeShared());
        pass_y.filter(pc);
        pass_z.setInputCloud(pc.makeShared());
        pass_z.filter(pc);

        pcl::toROSMsg(pc, pointcloud_data_pub);
        pointcloud_data_pub.header.stamp = ros::Time();
        pointcloud_data_pub.header.frame_id = "base_link";
        pointcloud_pub.publish(pointcloud_data_pub);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}