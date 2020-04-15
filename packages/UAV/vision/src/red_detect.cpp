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
#include <vision/redResult.h>

using namespace std;
using namespace cv;

//#define DEBUG_IMSHOW

vision::redResult red_result, det_result;
vision::redResult drone_status;
Mat img_sub;

bool inCenterFirst = false;
bool inCenterFirstFished = false;
bool inCenterSecond = false;
bool inCenterSecondFished = false;


// 中心时四个靶标位置记录
Point marks[4];
geometry_msgs::Point mark_center;
double x_sum=0, y_sum=0;
// 第二次拍照时确定的标靶位置
Point marks_second[4];

// ************************* 需要调节的参数 ******************************

//*调试用显示图像标志位
bool imgshow;
//detectRedMark
bool show_red_thres_image;
// 颜色阈参数
int RedLo0, RedLo1, RedLo2, RedHi0, RedHi1, RedHi2;


// >>>>>>>>detectLandAreaFirst()<<<<<<<<<
// 自适应二值化 区域大小(3,5,7,9)
bool thres_merge_first; //图像显示
int blockSize;  //  值越大噪声越大
bool region_result_fisrt; //显示图像结果


// >>>>>>>>>detectLandAreaSecond()<<<<<<
bool show_roi_img;
bool thres_merge_second;
int blockSize_second; // 值越大噪声越大

// 着陆区域大小
int landing_size;

// 自定义结构体
struct POINT
{
    Point topLeft;
    Point topRight;
    Point bottomLeft;
    Point bottomRight;
}roi_point;

enum fisrtDirection
{
    topLeft=1,
    bottomLeft,
    bottomRight,
    topRight
}direction;


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
    // cout << "in the callback" << endl;
}

void droneStatusCallback(const vision::redResult::ConstPtr &msg)
{
    drone_status = *msg;
    if (drone_status.inCenter)
    {
        if(inCenterFirstFished==false)
        {
            inCenterFirst = true;  // 该标志被置为true后不会再被置为false
        }else{
            inCenterSecond = true;
        }
    }

}

void detectRedMark(Mat &img, Scalar hsvRedLo, Scalar hsvRedHi)
{
    red_result.mark_ori.clear();
    // bgr 2 hsv
    Mat hsv_image;
    cvtColor(img, hsv_image, COLOR_BGR2HSV);

    vector<Mat> hsv;
    split(hsv_image, hsv);

    Mat red_thres_image;
    inRange(hsv_image, hsvRedLo, hsvRedHi, red_thres_image);

    // #ifdef DEBUG_IMSHOW
    // imshow("red_thres_image", red_thres_image);
    // #endif

    Mat result_image;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(red_thres_image, result_image, MORPH_OPEN, kernel);

    if(imgshow && show_red_thres_image)
    {
        imshow("result_image", result_image);
    }
    
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(result_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    // cout<<"contours num:" << contours.size()<< endl;
    if (contours.size() < 0)
    {
        cout << "NO CONTOUR" << endl;
        red_result.red_mark_num = 0;
    }

    red_result.red_mark_num = contours.size();
    vector<Moments> m(contours.size());
    // vector<Point2f> mark_ori(contours.size());
    
    geometry_msgs::Point mark_p;
    //清空累加计数
    x_sum=0;
    y_sum=0;
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
        red_result.mark_ori.push_back(mark_p);
        if(!inCenterFirst)
        {
            x_sum += m[i].m10 / m[i].m00;
            y_sum += m[i].m01 / m[i].m00;
        }
    }

}


//********************第一次着陆区域检测**************************
// 采用rgb三通道图像， 将图像分为四个区域， 给出可着陆区域中心点
//
void detectLandAreaFirst(Mat &img, Point* marks, Point center)  //vector<Point> &marks_p
{
    Mat mask = Mat::zeros(img.size(), CV_8UC1);
    // 凸多边形拟合
    fillConvexPoly(mask, marks, 4, Scalar(255, 255, 255));
    // imshow("mask", mask);
    Mat image_masked;
    bitwise_and(img, img, image_masked, mask);
    // imshow("img", image_masked);


    // Mat img_hsv, canny, thres, blur;
    // cvtColor(image_masked, img_hsv, COLOR_BGR2HSV);
    // vector<Mat> hsv;
    // split(img_hsv, hsv);
    // imshow("h", hsv[0]);
    // imshow("s", hsv[1]);
    // imshow("v", hsv[2]);
    // GaussianBlur(hsv[0], blur, Size(15,15), 0, 0);
    // Canny(blur, canny, 5, 150, 5);
    // adaptiveThreshold(canny, thres, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, 2);
    // imshow("thres_h", thres);

    // GaussianBlur(hsv[1], blur, Size(15,15), 0, 0);
    // Canny(blur, canny, 5, 150, 5);
    // adaptiveThreshold(canny, thres, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, 2);
    // imshow("thres_s", thres);

    // GaussianBlur(hsv[2], blur, Size(15,15), 0, 0);
    // Canny(blur, canny, 5, 150, 5);
    // adaptiveThreshold(canny, thres, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, 2);
    // imshow("thres_v", thres);

    Mat canny, thres, blur;
    Mat thres_r, thres_g, thres_b;
    vector<Mat> bgr;
    
    split(image_masked, bgr);
    // imshow("b", bgr[0]);
    // imshow("g", bgr[1]);
    // imshow("r", bgr[2]);
    GaussianBlur(bgr[0], blur, Size(9,9), 0, 0);
    // Canny(blur, canny, 5, 150, 5);
    // imshow("canny_b", canny);
    adaptiveThreshold(blur, thres, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, blockSize, 2);
    // imshow("thres_b", thres);
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(thres, thres_b, MORPH_CLOSE, element);
    // imshow("thres_b_after", thres_b);

    GaussianBlur(bgr[1], blur, Size(9,9), 0, 0);
    // Canny(blur, canny, 5, 150, 5);
    // imshow("canny_g", canny);
    adaptiveThreshold(blur, thres, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, blockSize, 2);
    // imshow("thres_g", thres);
    morphologyEx(thres, thres_g, MORPH_CLOSE, element);
    // imshow("thres_g_after", thres_g);

    GaussianBlur(bgr[2], blur, Size(9,9), 0, 0);
    // Canny(blur, canny, 5, 150, 5);
    // imshow("canny_r", canny);
    adaptiveThreshold(blur, thres, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, blockSize, 2);
    // imshow("thres_r", thres);
    morphologyEx(thres, thres_r, MORPH_CLOSE, element);
    // imshow("thres_r_after", thres_r);
    Mat thres_merge;
    bitwise_and(thres_r, thres_g, thres_merge);
    bitwise_and(thres_merge, thres_b, thres_merge);
    // 白色是轮廓
    threshold(thres_merge, thres_merge, 100, 255, THRESH_BINARY_INV);

    // #ifdef DEBUG_IMSHOW
    // imshow("thres_merge", thres_merge);
    // #endif

    if(imgshow && thres_merge_first)
    {
       imshow("thres_merge", thres_merge); 
    }
    // imshow("thres_merge", thres_merge);
    // waitKey(1);

    int img_height_half = (int)(img.size().height / 2);
    int img_width_half = (int)(img.size().width / 2);
    Mat Kernel = Mat::ones(Size(img_width_half, img_height_half), CV_8UC1);
    // cout << "kernel size" << Kernel.rows <<  Kernel.channels() << endl;
    Rect rect_topLeft(0, 0, img_width_half, img_height_half);
    Rect rect_topRight = rect_topLeft + Point(img_width_half, 0);
    Rect rect_bottomLeft = rect_topLeft + Point(0, img_height_half);
    Rect rect_bottomRight = rect_topLeft + Point(img_width_half, img_height_half);

    Mat img_topLeft = thres_merge(rect_topLeft);
    Mat img_topRight = thres_merge(rect_topRight);
    Mat img_bottomLeft = thres_merge(rect_bottomLeft);
    Mat img_bottomRight = thres_merge(rect_bottomRight);

    #ifdef DEBUG_IMSHOW
    imshow("img_topLeft", img_topLeft);
    imshow("img_topRight", img_topRight);
    imshow("img_bottomLeft", img_bottomLeft);
    imshow("img_bottomRight", img_bottomRight);
    #endif

    vector<double> scores;
    double score_topLeft = img_topLeft.dot(Kernel) / 255;
    scores.push_back(score_topLeft);
    double score_topRight = img_topRight.dot(Kernel) / 255;
    scores.push_back(score_topRight);
    double score_bottomLeft = img_bottomLeft.dot(Kernel) / 255;
    scores.push_back(score_bottomLeft);
    double score_bottomRight = img_bottomRight.dot(Kernel) / 255;
    scores.push_back(score_bottomRight);
    // cout << "score_topLeft: " << score_topLeft << endl;
    // cout << "score_topRight: " << score_topRight << endl;
    // cout << "score_bottomLeft: " << score_bottomLeft << endl;
    // cout << "score_bottomRight: " << score_bottomRight << endl;

    double score_min = 9999999;
    for (auto it=scores.begin(); it!=scores.end(); it++)
    {
        if (*it < score_min)
        {
            score_min = *it;
        }
    }
    // cout << "score_min: " << score_min << endl;

    geometry_msgs::Point target_point;
    if(score_min == score_topLeft)
    {
        target_point.x = (center.x + roi_point.topLeft.x) / 2;
        target_point.y = (center.y + roi_point.topLeft.y) / 2;
        direction = topLeft;
    }
    if(score_min == score_topRight)
    {
        target_point.x = (center.x + roi_point.topRight.x) / 2;
        target_point.y = (center.y + roi_point.topRight.y) / 2;
        direction = topRight;
    }
    if(score_min == score_bottomLeft)
    {
        target_point.x = (center.x + roi_point.bottomLeft.x) / 2;
        target_point.y = (center.y + roi_point.bottomLeft.y) / 2;
        direction = bottomLeft;
    }
    if(score_min == score_bottomRight)
    {
        target_point.x = (center.x + roi_point.bottomRight.x) / 2;
        target_point.y = (center.y + roi_point.bottomRight.y) / 2;
        direction = bottomRight;
    }

    det_result.mark_ori.push_back(target_point);

    if(imgshow && region_result_fisrt)
    circle(image_masked, Point(target_point.x, target_point.y), 4, Scalar(0, 0, 255), -1, 8, 0);
    imshow("img_detect", image_masked);


}


void detectLandAreaSecond(Mat& img)
{
    det_result.mark_ori.clear();
    int kernel_half = (landing_size-1)/2;
    int red_num = (int)(red_result.red_mark_num);
    int img_height_half = (int)(img.size().height / 2);
    int img_width_half = (int)(img.size().width / 2);
    Mat roi_img;
    det_result.mark_ori = red_result.mark_ori;
    if(red_num != 0) 
    {
        // marks_second[0]为左上角，逆时针排列
        if(red_num == 1 && direction == topLeft)
        {
            marks_second[0] = Point(red_result.mark_ori[0].x, red_result.mark_ori[0].y);
            marks_second[1] = Point(red_result.mark_ori[0].x, 2 * img_height_half);
            marks_second[2] = Point(2*img_width_half, 2 * img_height_half);
            marks_second[3] = Point(2*img_width_half, red_result.mark_ori[0].y);
        }
        if(red_num == 1 && direction == topRight)
        {
            marks_second[3] = Point(red_result.mark_ori[0].x, red_result.mark_ori[0].y);
            marks_second[0] = Point(0, red_result.mark_ori[0].y);
            marks_second[1] = Point(0, 2*img_height_half);
            marks_second[2] = Point(red_result.mark_ori[0].x, 2*img_height_half);
        }
        if(red_num == 1 && direction == bottomLeft)
        {
            marks_second[1] = Point(red_result.mark_ori[0].x, red_result.mark_ori[0].y);
            marks_second[2] = Point(2*img_width_half, red_result.mark_ori[0].y);
            marks_second[3] = Point(2*img_width_half, 0);
            marks_second[0] = Point(red_result.mark_ori[0].x, 0);
        }
        if(red_num == 1 && direction == bottomRight)
        {
            marks_second[2] = Point(red_result.mark_ori[0].x, red_result.mark_ori[0].y);
            marks_second[3] = Point(red_result.mark_ori[0].x, 0);
            marks_second[0] = Point(0, 0);
            marks_second[1] = Point(0, red_result.mark_ori[0].y);
        }

        // Mat mask = Mat::zeros(img.size(), CV_8UC1);
        // // 凸多边形拟合
        // fillConvexPoly(mask, marks_second, 4, Scalar(255, 255, 255));
        // // imshow("mask", mask);
        // Mat image_result;
        // bitwise_and(img, img, image_result, mask);
        // imshow("img", image_result);

    }
    else
    {   
        marks_second[0] = Point(0, 0);
        marks_second[1] = Point(0, 2*img_height_half);
        marks_second[2] = Point(2*img_width_half, 2*img_height_half);
        marks_second[3] = Point(2*img_width_half, 0);
    }

    Rect rect_roi(marks_second[0].x, marks_second[0].y, 
                 (int)(marks_second[2].x - marks_second[0].x),
                 (int)(marks_second[2].y - marks_second[0].y));
    roi_img = img(rect_roi);

    if(imgshow && show_roi_img)
    {
        imshow("roi_img", roi_img);
    }
    
    Mat canny, thres, blur;
    Mat thres_r, thres_g, thres_b;
    vector<Mat> bgr;
    
    split(roi_img, bgr);
    GaussianBlur(bgr[0], blur, Size(9,9), 0, 0);
    adaptiveThreshold(blur, thres, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, blockSize_second, 2);
    // imshow("thres_b", thres);
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(thres, thres_b, MORPH_CLOSE, element);
    // imshow("thres_b_after", thres_b);

    GaussianBlur(bgr[1], blur, Size(9,9), 0, 0);
    adaptiveThreshold(blur, thres, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, blockSize_second, 2);
    // imshow("thres_g", thres);
    morphologyEx(thres, thres_g, MORPH_CLOSE, element);
    // imshow("thres_g_after", thres_g);

    GaussianBlur(bgr[2], blur, Size(9,9), 0, 0);
    adaptiveThreshold(blur, thres, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, blockSize_second, 2);
    // imshow("thres_r", thres);
    morphologyEx(thres, thres_r, MORPH_CLOSE, element);
    // imshow("thres_r_after", thres_r);
    Mat thres_merge;
    bitwise_and(thres_r, thres_g, thres_merge);
    bitwise_and(thres_merge, thres_b, thres_merge);
    // 白色是轮廓
    threshold(thres_merge, thres_merge, 100, 255, THRESH_BINARY_INV);
    // imshow("second_merge", thres_merge);

    medianBlur(thres_merge,thres_merge, 9);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
    dilate(thres_merge, thres_merge, kernel, Point(-1, -1), 2);
    imshow("second_merge_dilate", thres_merge);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(thres_merge, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    for(size_t i=0; i<contours.size(); i++)
    {
        drawContours(thres_merge, contours, i, 255, FILLED, 8, hierarchy);
    }

    erode(thres_merge, thres_merge, kernel, Point(-1, -1), 2);
    imshow("second_merge_erode", thres_merge);

    if(imgshow && thres_merge_second)
    {
        imshow("thres_merge_second", thres_merge);
    }
    

    Mat landing_kernel = Mat::ones(Size(landing_size, landing_size), CV_8UC1);
    Mat result_img;
    Mat filter_img = thres_merge /255;

    filter2D(filter_img, result_img, CV_32F, landing_kernel);
    // imshow("img_result", result_img);

    // cout << "kernel_half: " << kernel_half << endl;
    cout << "result_img size " << result_img.cols << "*" << result_img.rows << endl;

    Rect result_rect(kernel_half, kernel_half, result_img.cols - 2*kernel_half, result_img.rows - 2*kernel_half);
    double minVal, maxVal;
    Point minLoc, maxLoc;
    Mat result_img_mini = result_img(result_rect);
    cout << "result_img_mini size " << result_img_mini.cols << "*" << result_img_mini.rows << endl;


    vector<Point> landing_point;
    minMaxLoc(result_img_mini, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    // cout << "min location " << minLoc.x + result_rect.tl().x << " " << minLoc.y + result_rect.tl().y <<endl;
    landing_point.push_back(Point(minLoc.x + result_rect.tl().x, minLoc.y + result_rect.tl().y));
    // result_img_mini.at<float>(minLoc.x, minLoc.y) = maxVal;


    geometry_msgs::Point landing_point_raw;
    landing_point_raw.x = landing_point[0].x + rect_roi.tl().x;
    landing_point_raw.y = landing_point[0].y + rect_roi.tl().y;

    rectangle(img, Point(landing_point_raw.x-kernel_half, landing_point_raw.y-kernel_half),
              Point(landing_point_raw.x + kernel_half, landing_point_raw.y + kernel_half),
              Scalar(255, 255, 255), 2, 4);
    imshow("img", img);

    det_result.mark_ori.push_back(landing_point_raw);



}

int main(int argc, char** argv)
{
    cout << "opencv version" << endl;
    cout << CV_VERSION << endl;

    ros::init(argc, argv, "planet_landmark_detect");
    ros::NodeHandle nh;
    ros::Rate rate_ = ros::Rate(20.0);

    // ******************参数读取*******************************
    // 色域参数
    nh.param<int>("RedLo0", RedLo0, 156);
    nh.param<int>("RedLo1", RedLo1, 43);
    nh.param<int>("RedLo2", RedLo2, 46);

    nh.param<int>("RedHi0", RedHi0, 180);
    nh.param<int>("RedHi1", RedHi1, 255);
    nh.param<int>("RedHi2", RedHi2, 255);

    nh.param<int>("blockSize", blockSize, 7);
    nh.param<int>("blockSize_second", blockSize_second, 5);

    nh.param<int>("landing_size", landing_size, 61);

    nh.param<bool>("imgshow", imgshow, true);
    nh.param<bool>("show_red_thres_image", show_red_thres_image, false);
    nh.param<bool>("thres_merge_first", thres_merge_first, false);
    nh.param<bool>("show_roi_img", show_roi_img, false);
    nh.param<bool>("thres_merge_second", thres_merge_second, false);
    nh.param<bool>("region_result_fisrt", region_result_fisrt, false);

    //红色区域
    const Scalar hsvRedLo(RedLo0, RedLo1, RedLo2);
    const Scalar hsvRedHi(RedHi0, RedHi1, RedHi2);

    

    cout << "hsvRedLo " << hsvRedLo << "hsvRedHi " << hsvRedHi <<endl;

    //【发布】处理结果
    ros::Publisher redResult_pub = nh.advertise<vision::redResult>("planet_landing/red_result", 100);
    ros::Publisher detResult_pub = nh.advertise<vision::redResult>("planet_landing/det_result", 100);

    // 【订阅】无人机状态
    ros::Subscriber drone_status_sub = nh.subscribe<vision::redResult>("planet_landing/det_start", 100, droneStatusCallback);

    //image_transport 负责图像发布与订阅
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/iris/usb_cam2/image_raw", 1, imageCallback);
    image_transport::Publisher pub = it.advertise("planet_landing/image/detect_result", 1);
    Mat image;

    while(img_sub.empty())
    {
        cout<<"no image data!!"<<endl;
        ros::spinOnce();
        rate_.sleep();
        continue;
    }
    int in_center_count = 0;

    while (ros::ok())
    {   
        Mat src_image;
        img_sub.copyTo(image);
        image.copyTo(src_image);

        detectRedMark(image, hsvRedLo, hsvRedHi);

        if(!inCenterFirst) //查找四个靶标中心
        {
            //计算已检测到的靶标中心
            mark_center.x = x_sum / red_result.red_mark_num;
            mark_center.y = y_sum / red_result.red_mark_num;
            red_result.mark_ori.push_back(mark_center);
            circle(image, Point2f(mark_center.x, mark_center.y), 4, Scalar(0, 0, 255), -1, 8, 0);
            #ifdef DEBUG_IMSHOW
            imshow("first_detect_image", image);
            #endif
            // for(auto it=0; it < 5; it++)
            // {
            //     cout << "point\n" << red_result.mark_ori[it] << endl;
            // }
        }
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(msg);
        redResult_pub.publish(red_result);

        
        if(inCenterFirst && inCenterFirstFished==false)  // 沙盘中心第一次拍照
        {
            det_result.red_mark_num = red_result.red_mark_num;
            det_result.mark_ori = red_result.mark_ori;

            in_center_count++;
            Point center = Point((int)(mark_center.x), (int)(mark_center.y));

            // 逆序排列靶标位置  -> 左上 -> 左下 -> 右下 -> 右上
            for(int i = 0; i < 4; i ++)
            {
                if(red_result.mark_ori[i].x < center.x && red_result.mark_ori[i].y < center.y)
                {
                    roi_point.topLeft = Point(red_result.mark_ori[i].x, red_result.mark_ori[i].y);
                    marks[0] = roi_point.topLeft;
                }
                if(red_result.mark_ori[i].x > center.x && red_result.mark_ori[i].y < center.y)
                {
                    roi_point.topRight = Point(red_result.mark_ori[i].x, red_result.mark_ori[i].y);
                    marks[3] = roi_point.topRight;
                }
                if(red_result.mark_ori[i].x < center.x && red_result.mark_ori[i].y > center.y)
                {
                    roi_point.bottomLeft = Point(red_result.mark_ori[i].x, red_result.mark_ori[i].y);
                    marks[1] = roi_point.bottomLeft;
                }
                if(red_result.mark_ori[i].x > center.x && red_result.mark_ori[i].y > center.y)
                {
                    roi_point.bottomRight = Point(red_result.mark_ori[i].x, red_result.mark_ori[i].y);
                    marks[2] = roi_point.bottomRight;
                }
            }

            detectLandAreaFirst(src_image, marks, center);
            // if(in_center_count > 10)
            // {
            //     // inCenterFirst = false;
            //     inCenterFirstFished = true;    
            // } 
            inCenterFirstFished = true; 

            detResult_pub.publish(det_result); 
        }

        if (inCenterSecond && inCenterSecondFished == false)
        {
            detectLandAreaSecond(src_image);
            detResult_pub.publish(det_result);
            inCenterSecondFished = true;
        }

        if(imgshow)
        {
            int c = waitKey(1);
            if( (char)c == 27 )
                break;
        }

        cout << "**************************************"<<endl;
        cout << "inCenterFirst       : " << inCenterFirst << endl;
        cout << "inCenterFirstFished : " << inCenterFirstFished << endl;
        cout << "inCenterSecond      : " << inCenterSecond << endl;
        cout << "num of mark         : " << (int)(red_result.red_mark_num) << endl;
        cout << "direction           : " << direction << endl;
        cout << "--------------处理状态:"<<endl;
        if(!inCenterFirst)
            cout << "           查找四个标靶中心------------" << endl;
        if(inCenterFirst && inCenterFirstFished==false)
            cout << "           沙盘中心第一次拍照------------" << endl;
        if(inCenterSecond)
            cout << "           区域第二次拍照------------" << endl;
        

        ros::spinOnce();
        rate_.sleep();
    }
    
}
