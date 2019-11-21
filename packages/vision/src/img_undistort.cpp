/*****************************************************
 * img_undistort.cpp
 * 
 * Author: chow
 * 
 * Update Time: 2019.11.08
 * 
 * 说明:图像发布
 * 1.矫正鱼眼相机图像
 * 2.需要相机内参, 畸变系数(四个数)
 * 
 * ***************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace cv;
using namespace std;

//定义全局变量
cv::Mat image;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{ 
  try
  {
    image = cv_bridge::toCvCopy(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

Mat imageUndistort(Mat &src)
{
   cv::Size img_size;
   img_size.width = 848;
   img_size.height = 800;
   cv::Mat distortiona = src.clone();
   cv::Mat camera_matrixa = (cv::Mat_<double>(3, 3) << 283.326110, 0.0, 435.7886, 0, 284.37469, 395.31988, 0, 0, 1);
   cv::Mat distortion_coefficientsa=(cv::Mat_<double >(1,4)<<0.000768995,0.0303144,-0.028857899,0.00372209);
   cv::fisheye::undistortImage(src, distortiona, camera_matrixa, distortion_coefficientsa,camera_matrixa,img_sizea);
   return distortiona;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ros::Rate rate(20);

  image_transport::ImageTransport it(nh);
  //【订阅】 订阅T265图像数据，鱼眼相机fisheye2
  image_transport::Subscriber sub = it.subscribe("/camera/fisheye2/image_raw", 1, imageCallback);
  //【发布】 发布矫正后的鱼眼相机图像
  image_transport::Publisher pub = it.advertise("/camera/fisheye2/image_undistort", 1);
  
  Mat img_undistort;
  while(image.empty())
  {
    cout<<"wait for image data"<<endl;
    ros::spinOnce();
    rate.sleep();
    continue;
   }
  while(ros::ok())
  {
   cout<<"have data"<<endl;
   img_undistort = imageUndistort(image);
   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_undistort).toImageMsg();
   pub.publish(msg);
   ros::spinOnce();
   rate.sleep();
  }

}

