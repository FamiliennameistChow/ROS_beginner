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
# include <pcl/filters/statistical_outlier_removal.h>
# include <pcl/keypoints/uniform_sampling.h>


// 参考
// http://docs.ros.org/jade/api/pcl_ros/html/namespacepcl__ros.html#a29cf585a248dc53517834f4c5a1c4d69

using namespace std;

sensor_msgs::PointCloud2 pointcloud_data_pub;
sensor_msgs::PointCloud2 pointcloud_data_sub;
sensor_msgs::PointCloud2 pointcloud_data_sub_process;
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sub (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pub (new pcl::PointCloud<pcl::PointXYZ>);

void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg){

    // pcl::fromROSMsg (*msg, *cloud_sub);//cloud is the output
    pointcloud_data_sub = *msg;
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "tf_listner");

    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    tf::TransformListener tf_listener;

    ros::Rate rate(10.0);
    std::string pointcloud_trans_topic, pointcloud_in_topic;
    private_nh.param<std::string>("pointcloud_trans_topic", pointcloud_trans_topic, std::string("/kinect/depth/points_global"));
    private_nh.param<std::string>("pointcloud_in_topic", pointcloud_in_topic, std::string("/kinect/depth/points"));


    ros::Subscriber pointcloud_sub = n.subscribe<sensor_msgs::PointCloud2>(pointcloud_in_topic, 100, pointcloud_cb);
    ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>(pointcloud_trans_topic, 100);

    // 等待获取监听信息tf map和camera_link
    tf_listener.waitForTransform("world", "camera_link", ros::Time(0), ros::Duration(3.0));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_data_sub_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_uniformsampled_ (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ (new pcl::PointCloud<pcl::PointXYZRGB>);

    while(ros::ok())
    {   
        pointcloud_data_sub.data = pointcloud_data_sub.data;
        pointcloud_data_sub.header.frame_id = std::string("camera_link");
        cout<< "header: " << pointcloud_data_sub.header.frame_id << endl;
        cout<< "timer: " << pointcloud_data_sub.header.stamp << endl;
        // cout<< "is_desece: " << pointcloud_data_sub.is_dense << endl;
        // pointcloud_data_pub.header.frame_id = std::string("map");
        try{
            
            
            pcl::fromROSMsg(pointcloud_data_sub, *pointcloud_data_sub_pc);


            // 均匀采样，取一定半径 r 的球体内的点保留质心。
            pcl::UniformSampling<pcl::PointXYZRGB> filter;
            filter.setInputCloud(pointcloud_data_sub_pc);
            filter.setRadiusSearch(0.03);
            // We need an additional object to store the indices of surviving points.
            pcl::PointCloud<int> keypointIndices;
            filter.compute(keypointIndices);
            pcl::copyPointCloud(*pointcloud_data_sub_pc, keypointIndices.points, *cloud_uniformsampled_);
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

            pcl::toROSMsg(*cloud_filtered_, pointcloud_data_sub_process);
            // pointcloud_data_sub_process.header.frame_id = std::string("camera_link");
            // pointcloud_data_sub_process.header.stamp = pointcloud_data_sub.header.stamp;
            // cout << "------" << endl;
            // http://docs.ros.org/jade/api/pcl_ros/html/namespacepcl__ros.html#a29cf585a248dc53517834f4c5a1c4d69
            pcl_ros::transformPointCloud("world", pointcloud_data_sub_process, pointcloud_data_pub, tf_listener);
            // pcl::toROSMsg(*cloud_pub, pointcloud_data_pub);
            // cout << "++++++++" << endl;
            pointcloud_data_pub.header.frame_id = std::string("world");
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