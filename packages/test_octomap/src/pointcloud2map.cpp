# include <ros/ros.h>
# include <tf/transform_listener.h>
# include <geometry_msgs/PointStamped.h>
# include <sensor_msgs/PointCloud2.h>
# include <iostream>
# include <pcl_ros/point_cloud.h>
# include <pcl_ros/transforms.h>
# include <pcl_ros/impl/transforms.hpp>
// # include <pcl/point_types.h>
# include <pcl_conversions/pcl_conversions.h>

using namespace std;

sensor_msgs::PointCloud2 pointcloud_data_pub;
sensor_msgs::PointCloud2 pointcloud_data_sub;
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sub (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub (new pcl::PointCloud<pcl::PointXYZ>);

void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg){

    // pcl::fromROSMsg (*msg, *cloud_sub);//cloud is the output
    pointcloud_data_sub = *msg;
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "tf_listner");

    ros::NodeHandle n;

    tf::TransformListener tf_listener;

    ros::Rate rate(10.0);
    std::string pointcloud_trans_topic;
    n.param<std::string>("pointcloud_trans_topic", pointcloud_trans_topic, std::string("/camera/depth/points_global"));

    ros::Subscriber pointcloud_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 100, pointcloud_cb);
    ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>(pointcloud_trans_topic, 100);

    // 等待获取监听信息mapk和camera_link
    tf_listener.waitForTransform("map", "camera_link", ros::Time(0), ros::Duration(3.0));

    while(ros::ok())
    {   
        pointcloud_data_sub.data = pointcloud_data_sub.data;
        pointcloud_data_sub.header.frame_id = std::string("camera_link");
        cout<< "header: " << pointcloud_data_sub.header.frame_id << endl;
        cout<< "timer: " << pointcloud_data_sub.header.stamp << endl;
        // cout<< "is_desece: " << pointcloud_data_sub.is_dense << endl;
        // pointcloud_data_pub.header.frame_id = std::string("map");
        try{
            cout << "------" << endl;
            // http://docs.ros.org/jade/api/pcl_ros/html/namespacepcl__ros.html#a29cf585a248dc53517834f4c5a1c4d69
            pcl_ros::transformPointCloud("map", pointcloud_data_sub, pointcloud_data_pub, tf_listener);
            // pcl::toROSMsg(*cloud_pub, pointcloud_data_pub);
            cout << "++++++++" << endl;
            cout<< "pub header: " << pointcloud_data_pub.header.frame_id << endl;
            pointcloud_pub.publish(pointcloud_data_pub);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}