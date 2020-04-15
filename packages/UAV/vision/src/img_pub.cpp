# include <ros/ros.h>
# include <image_transport/image_transport.h>
# include <opencv2/highgui/highgui.hpp>
# include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

// -----------------发布前置摄像头数据----------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_pub");
    ros::NodeHandle nh;
    ros::Rate rate_ = ros::Rate(20.0);

    //image_transport 负责发布与订阅
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image_pub", 1);
  
    //0为读取摄像头
    VideoCapture cam(0);
    if(!cam.isOpened())
    {
        cerr << "the front-facing camera is NOT open!!!"<< endl;
        return 1;
    }

    Mat frame;

    while (ros::ok())
    {
        cam >> frame;
        if (frame.empty())
        {
            ROS_ERROR_STREAM("Failed to capture image!!");
            ros::shutdown();
        }

        //将图像从cv::Mat类型转化成sensor_msgs/Image类型并发布
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        /*
        cv_bridge可以有选择的对颜色和深度信息进行转化。为了使用指定的特征编码，就有下面集中的编码形式：

        mono8:  CV_8UC1， 灰度图像
        mono16: CV_16UC1,16位灰度图像
        bgr8: CV_8UC3,带有颜色信息并且颜色的顺序是BGR顺序
        rgb8: CV_8UC3,带有颜色信息并且颜色的顺序是RGB顺序
        bgra8: CV_8UC4, BGR的彩色图像，并且带alpha通道
        rgba8: CV_8UC4,CV，RGB彩色图像，并且带alpha通道
        */

        pub.publish(msg);
        ros::spinOnce();
        rate_.sleep(); 
    }
}