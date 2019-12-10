/***************************************************************************************************************************
 * red_detect.cpp
 *
 * Author: Chow
 *
 * Update Time: 2019.12.10
 *
 * 说明: 红色地标识别程序(无人机着陆大赛)
 *      1. 【订阅】下置摄像头图像数据
 *      2. 【发布】识别结果 <vision::redResult> 
 *      2. 【发布】识别后的图像 "auto_landing/image/detect_result" 
***************************************************************************************************************************/
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <drone_flight_modes.hpp>
#include <vision/redResult.h>

using namespace std;
using namespace cv;

// #define DEBUG_IMSHOW

vision::redResult det_result;
Mat img_sub;
//红色区域
const Scalar hsvRedLo(156, 43, 46);
const Scalar hsvRedHi(180, 255, 255);


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
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

void detect4RedMark(Mat &img)
{
    det_result.mark_ori.clear();
    // bgr 2 hsv
    Mat hsv_image;
    cvtColor(img, hsv_image, COLOR_BGR2HSV);

    vector<Mat> hsv;
    split(hsv_image, hsv);

    Mat red_thres_image;
    inRange(hsv_image, hsvRedLo, hsvRedHi, red_thres_image);

    #ifdef DEBUG_IMSHOW
    imshow("red_thres_image", red_thres_image);
    #endif

    Mat result_image;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(red_thres_image, result_image, MORPH_OPEN, kernel);
    #ifdef DEBUG_IMSHOW
    imshow("result_image", result_image);
    #endif

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(result_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    cout<<"contours num:" << contours.size()<< endl;
    if (contours.size() < 0)
    {
        cout << "NO CONTOUR" << endl;
        det_result.red_mark_num = 0;
    }

    det_result.red_mark_num = contours.size();
    vector<Moments> m(contours.size());
//    vector<Point2f> mark_ori(contours.size());
    double x_sum=0, y_sum=0;
    geometry_msgs::Point mark_p, mark_center;
    for(size_t i = 0; i<contours.size(); i++)
    {
        // cout << "====" << endl;
        Scalar color(rand() & 255, rand() & 255, rand()&255);
        drawContours(img, contours, i, color, FILLED, 8, hierarchy);
        // cout << "----" << endl;
        m[i] = moments(contours[i], false);
        // cout << "++++" << endl;
        mark_p.x =  m[i].m10 / m[i].m00;
        mark_p.y =  m[i].m01 / m[i].m00;
        det_result.mark_ori.push_back(mark_p);
        x_sum += m[i].m10 / m[i].m00;
        y_sum += m[i].m01 / m[i].m00;
    }

    mark_center.x = x_sum / contours.size();
    mark_center.y = y_sum / contours.size();
    det_result.mark_ori.push_back(mark_center);
    circle(img, Point2f(mark_center.x, mark_center.y), 4, Scalar(0, 0, 255), -1, 8, 0);
    #ifdef DEBUG_IMSHOW
    imshow("raw_image", img);
    #endif
    for(auto it=0; it < 5; it++)
    {
        cout << "point\n" << det_result.mark_ori[it] << endl;
    }
}

int main(int argc, char** argv)
{
    cout << "opencv version" << endl;
    cout << CV_VERSION << endl;

    ros::init(argc, argv, "planet_landmark_detect");
    ros::NodeHandle nh;
    ros::Rate rate_ = ros::Rate(20.0);

    //【发布】处理结果
    ros::Publisher redResult_pub = nh.advertise<vision::redResult>("planet_landing/redResult", 100);

    //image_transport 负责图像发布与订阅
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/iris/usb_cam_down/image_raw", 1, imageCallback);
    image_transport::Publisher pub = it.advertise("planet_landing/image/detect_result", 1);
    Mat image;

    while(img_sub.empty())
    {
        cout<<"no image data!!"<<endl;
        ros::spinOnce();
        rate_.sleep();
        continue;
    }

    while (ros::ok())
    {   
        img_sub.copyTo(image);
        // 检测四个红色靶标
        detect4RedMark(image);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(msg);
        redResult_pub.publish(det_result);

        #ifdef DEBUG_IMSHOW
        int c = waitKey(1);
        if( (char)c == 27 )
            break;
        #endif

        ros::spinOnce();
        rate_.sleep();
    }
    
}
