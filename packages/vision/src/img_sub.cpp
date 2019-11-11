#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/bind.hpp>

using namespace cv;
using namespace std;
Mat img_sub;
cv_bridge::CvImagePtr cv_ptr;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 try
 {
  //  cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
   cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
 }
 catch (cv_bridge::Exception& e)
 {
   ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
 }
//  img_sub = cv_ptr->image;
cv_ptr->image.copyTo(img_sub);
 cout << "in the callback" << endl;
}



int main(int argc, char **argv)
{
 ros::init(argc, argv, "image_sub");
 ros::NodeHandle nh;
 ros::Rate rate_ = ros::Rate(20.0);
//  cv::namedWindow("view");
//  cv::startWindowThread();
 image_transport::ImageTransport it(nh);
 image_transport::Subscriber sub = it.subscribe("camera/image_pub", 1, imageCallback);
 
 while(img_sub.empty())
 {
   cout<<"no image data!!"<<endl;
   ros::spinOnce();
   rate_.sleep();
   continue;
 }


 while(ros::ok())
 {
 cv::imshow("img", img_sub);
 cv::waitKey(1);
 ros::spinOnce();
 rate_.sleep();
 }
//  cv::destroyWindow("view");
}


