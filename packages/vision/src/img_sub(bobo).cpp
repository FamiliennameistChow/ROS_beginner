//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

//using namespace cv;
//using namespace std;
//Mat img_sub;
//void imageCallback(const sensor_msgs::ImageConstPtr& msg)
//{
//  cv_bridge::CvImagePtr cv_ptr;
//  try
//  {
//    cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
//  }
//  catch (cv_bridge::Exception& e)
//  {
//    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//  }
//  img_sub = cv_ptr->image;
//}


// 
//int main(int argc, char **argv)
//{
//  ros::init(argc, argv, "image_listener");
//  ros::NodeHandle nh;
////  cv::namedWindow("view");
////  cv::startWindowThread();
//  image_transport::ImageTransport it(nh);
//  image_transport::Subscriber sub = it.subscribe("camera/image_front_facing", 1, imageCallback);

//  while(ros::ok())
//  {
//  if(img_sub.empty())
//  {
//    cout<<"no image data!!"<<endl;
//    continue;
//  }
//  cv::imshow("img", img_sub);
//  cv::waitKey(1);
//  ros::spin();
//  }
////  cv::destroyWindow("view");
//}

#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

cv::Mat image;
// cv_bridge::CvImagePtr cv_ptr;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{ 
  try
  {
    // image = cv_bridge::toCvShare(msg, "bgr8")->image;
    image = cv_bridge::toCvCopy(msg,"bgr8")->image;
 cv::imshow("view", image);
//  cvWaitKey(1);
//   cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
//   cvWaitKey(1);
    
 
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_sub");
  ros::NodeHandle nh;
//  cv::namedWindow("view", 0);
//  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image_pub", 1, imageCallback);
  while(image.empty())
  {
    cout << "wait for image" << endl;
  }
  while(ros::ok())
  {
   cv::imshow("view", image);
   cvWaitKey(1);
   ros::spin();
  }
    ros::spin();
//  cv::destroyWindow("view");
}

