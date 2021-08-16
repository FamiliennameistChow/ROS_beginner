#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <pcl/point_types.h>                 //pcl点云格式头文件
#include <pcl_conversions/pcl_conversions.h> //转换?
#include <pcl/point_cloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
using namespace std;

double leng = 0.0;
pcl::PointXYZI pt_last;


void Callback(const sensor_msgs::PointCloud2& msg){
	//cout<<"height:"<<msg->height<<"    width:"<<msg->width<<endl;
	//解析data数据
	//sensor_msgs::PointCloud convertCloud;
	//sensor_msgs::convertPointCloud2ToPointCloud(msg,convertCloud);
	//for(int i=0;i<convertCloud.points.size();i++)
	//{
	//	cout<<convertCloud.points[i]<<endl;
	//}
	//cout<<"--------------------------------------------------"<<endl;
	pcl::PointCloud<pcl::PointXYZI> cloud_pcl_xyzi;
	pcl::fromROSMsg(msg, cloud_pcl_xyzi);

    cout<< "in: " << cloud_pcl_xyzi.points[cloud_pcl_xyzi.points.size()-1]<<endl;
    leng += pow(pow(cloud_pcl_xyzi.points[cloud_pcl_xyzi.points.size()-1].x - pt_last.x ,2) + pow(cloud_pcl_xyzi.points[cloud_pcl_xyzi.points.size()-1].y - pt_last.y ,2)  ,0.5);
    pt_last = cloud_pcl_xyzi.points[cloud_pcl_xyzi.points.size()-1];
    cout<< "last: " << cloud_pcl_xyzi.points[cloud_pcl_xyzi.points.size()-1]<<endl;
	

    cout << " leng: " << leng << endl;
}

int main(int argc,char ** argv){
	ros::init(argc,argv,"cul_traj_len");
	ros::NodeHandle node;

    pt_last.x = 0.0;
    pt_last.y = 0.0;
    pt_last.z = 0.0;

	ros::Subscriber sub=node.subscribe("/key_pose_origin",100,Callback);
	ros::spin();
	return 0;
}
