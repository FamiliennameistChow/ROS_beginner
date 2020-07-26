//
//  main.cpp
//  Graduation
//
//  Created by NUC8i5BEH on 2020/3/17.
//  Copyright © 2020 NUC8i5BEH. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <math.h>
#include <vector>
#include <cstdlib>
#include <fstream>
#include <algorithm>
#include <opencv2/objdetect.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace cv;
using namespace std;

// #define DEBUG 1;


using std::cout;
using std::cerr;
using std::vector;
using std::string;

using cv::Mat;
using cv::Point2f;
using cv::KeyPoint;
using cv::Scalar;
using cv::Ptr;

using cv::FastFeatureDetector;
using cv::SimpleBlobDetector;

using cv::DMatch;
using cv::BFMatcher;
using cv::DrawMatchesFlags;
using cv::Feature2D;
using cv::ORB;
using cv::BRISK;
using cv::AKAZE;
using cv::KAZE;

using cv::xfeatures2d::BriefDescriptorExtractor;
using cv::xfeatures2d::SURF;
using cv::xfeatures2d::SIFT;
using cv::xfeatures2d::DAISY;
using cv::xfeatures2d::FREAK;


class MoonLanding{
public:
    MoonLanding(ros::NodeHandle nh, string desc_type, string match_type);
    ~MoonLanding();
    
    void detectCraters();
    void detectRocks();
    void searchLandArea();
    bool trackingLandmark();
    RotatedRect landArea();
    Point3f current_pose();
    
private:
    ros::NodeHandle nh_;
    ros::Rate rate_ = ros::Rate(40.0);

    Mat img_sub_;
    Mat img_map_, img_map_gray_, img_map_to_draw_;
    Mat img_camera_, img_camera_gray_;
    
    Mat craters_, rocks_, obstacles_;
    
    RotatedRect landArea_;
    
    string desc_type_, match_type_;
    
    vector<KeyPoint> kpts_map_;  // 作为地图的图片的关键点
    vector<KeyPoint> kpts_camera_;

    Mat desc_map_; // 作为地图的图片的关键点描述子
    Mat desc_camera_;
    
    vector<DMatch> matches_;
    vector<char> match_mask_;
    vector<Point2f> match_points_map_, match_points_camera_;

    vector<Point2f> average_pts_;
    double scale_;
    double angle_;
    
    Point2f camera_position_, camera_position_rotated_;
    
    Point3f current_pose_;
    Point3f camera_pose_last_;

    image_transport::Publisher match_img_pub_;
    
    const double kDistanceCoef_ = 2.0;
    const int kMaxMatchingSize_ = 30;
    const double kMinDistanceThreshold_ = 30.0;
    int kWrongPoseNum_ = 0;
    int kKeyFrameInterval_ = 25;
    int kSegNum = 0;

    boost::thread* thread_sub_img_ = nullptr;  // for receiving image
    void receiveImageThread();

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    void detect_and_compute(Mat& img, vector<KeyPoint>& kpts, Mat& desc);
    void match(Mat& desc1, Mat& desc2, vector<DMatch>& matches);
    void findKeyPointsHomography();

    void calcAveragePts();
    void calcScale();
    void calcAngle();
    
    void AdaptiveFindThreshold(cv::InputArray src, double *low, double *high, int aperture_size=3);
    void _AdaptiveFindThreshold(Mat *dx, Mat *dy, double *low, double *high);
};

MoonLanding::MoonLanding(ros::NodeHandle nh, string desc_type, string match_type)
    : nh_(nh), desc_type_(desc_type), match_type_(match_type)
{
    image_transport::ImageTransport it(nh_);
    match_img_pub_ = it.advertise("map_img_drawed", 1);

    thread_sub_img_ = new boost::thread(boost::bind(&MoonLanding::receiveImageThread, this));

    while(img_sub_.empty())
    {
        cout<<"No image data!!!"<<endl;
        ros::spinOnce();
        rate_.sleep();
    }
}

MoonLanding::~MoonLanding() {
    delete thread_sub_img_;
}


RotatedRect MoonLanding::landArea()
{
    return landArea_;
}

Point3f MoonLanding::current_pose()
{
    return current_pose_;
}

void MoonLanding::receiveImageThread()
{
    image_transport::ImageTransport it(nh_);
    image_transport::Subscriber sub = it.subscribe("/iris/usb_cam/image_raw", 1, boost::bind(&MoonLanding::imageCallback, this, _1));

    while (ros::ok())
    {
        ros::spinOnce();
        rate_.sleep();
    }
 
}


void MoonLanding::imageCallback(const sensor_msgs::ImageConstPtr& msg)
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

    cv_ptr->image.copyTo(img_sub_);
    // cout << "in the callback" << endl;
}

void MoonLanding::detectCraters()
{
    Mat gray_original, gray, edge, contour, allContour, circle;
        
    img_map_gray_.copyTo(circle);
    contour = Mat::zeros(img_map_gray_.rows, img_map_gray_.cols, CV_8UC1);
    allContour = Mat::zeros(img_map_gray_.rows, img_map_gray_.cols, CV_8UC1);
    
        
        
        //        // 限制对比度的自适应直方图均衡化（CLAHE）。观感较好的图片不需要此操作
    //        double clipLimit = 1.5;
    //        Ptr<cv::CLAHE> clahe = createCLAHE();
    //        clahe->setClipLimit(clipLimit);
    //        clahe->apply(src_gray, src_gray);
            
        //        equalizeHist(src_gray, gray);
        
    //    // 滤波
    //    GaussianBlur(gray_original, gray, Size(3,3), 2, 2);
    //    medianBlur(gray_original, gray, 5); //滤除脉冲、椒盐噪声
        bilateralFilter(img_map_gray_, gray, 7, 7*2, 7/2);
        //    blur(gray_original, gray, Size(5,5));

      
        // 自适应阈值的canny边缘检测
        //1 基于图像中值的阈值自适应
        Mat tmp = gray.reshape(1, 1);//make matrix new number of channels and new number of rows. here Put data: 1 row, all cols
        Mat sorted; //after sorted data
        cv::sort(tmp, sorted, SORT_ASCENDING);
        int median = sorted.at<uchar>(sorted.cols / 2);//find median data in median of cols
        double low_thresh = 0.3*median;  //0.66
        double high_thresh = 0.7*median;  //1.73
        cout << "median : " << median << endl;
        cout << "low_thresh : " << low_thresh << "  " << "high_thresh : " <<high_thresh << endl;
        double max = 0.8*sorted.at<uchar>(sorted.rows*sorted.cols-1);
        if(high_thresh > max)
        {
            high_thresh = 0.8*sorted.at<uchar>(sorted.rows*sorted.cols-1);
            low_thresh = high_thresh* 0.3/0.7;
        }
        cout << "after:  low_thresh : " << low_thresh << "  " << "high_thresh : " <<high_thresh << endl;
        Canny(gray, edge, low_thresh, high_thresh, 3);

        
        
    //    //2 基于梯度直方图的阈值自适应
    //    double low_thresh = 0, high_thresh = 0;
    //    AdaptiveFindThreshold(gray, &low_thresh, &high_thresh);
    //    Canny(gray, edge, low_thresh, high_thresh, 3);
        
        //进行形态学操作（闭运算，轮廓不繁杂时可用）
        //定义核
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(edge, edge, MORPH_CLOSE, element);
        
        //    element = getStructuringElement(MORPH_RECT, Size(3, 3));
        //    morphologyEx(edge, edge, MORPH_OPEN, element);
        
        //寻找轮廓
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(edge, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        #ifdef DEBUG
        ROS_INFO("after findContours");
        #endif
        
        // 通过轮廓大小和椭圆拟合来筛选轮廓
        for(int index = 0; index >= 0; index = hierarchy[index][0])
        {
            //画出查询到的所有轮廓
            drawContours(allContour, contours, index, Scalar::all(255), 1, 8, hierarchy);
            
            //筛选出包含点的数量大于某值的轮廓
            if( contours[index].size() < (img_map_gray_.rows/100 + img_map_gray_.cols/100) )  continue;
            // cout << "pass the size filter" << endl;
            
            //筛选出轮廓面积大于某值的轮廓
            double area = cv::contourArea(contours[index]);
            if (area < (img_map_gray_.rows * img_map_gray_.cols/100000))  continue;
            // cout << "pass the area filter" << endl;


            //筛选出弧线半径在某一范围内的轮廓
            double MinR = img_map_gray_.rows/100, MaxR = img_map_gray_.rows;
            double arc_length = cv::arcLength(contours[index], true);
            double radius = arc_length / (2 * M_PI);
            if (!(MinR < radius && radius < MaxR))  continue;
            // cout << "pass the length filter" << endl;
            
            // 1、获得最小包围圆
            Point2f center_circle;
            float radius_circle;
            minEnclosingCircle(contours[index], center_circle, radius_circle);
            
            
            
//            // 2、椭圆拟合
//            RotatedRect box = fitEllipse(contours[index]);
//            //如果长宽比大于某个阈值，则排除，不做拟合
//            if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*2 )  continue;
            
            // 画出符合筛选条件的的轮廓
            drawContours(contour, contours, index, Scalar::all(255), 1, 8, hierarchy);
            
//            // 2、画出拟合的椭圆
//            cv::circle(circle, box.center, 3, Scalar::all(255), -1, 8, 0); //画出圆心
//            ellipse(circle, box, Scalar::all(255), 2, LINE_8);
//            ellipse(mask_crater, box, Scalar::all(255), -1, LINE_8);

            // 1、画出最小包围圆
            cv::circle(circle, center_circle, 2, Scalar::all(255), -1, 8, 0); //画出圆心
            cv::circle(circle, center_circle, radius_circle, Scalar::all(255), 1, 8, 0);
            cv::circle(craters_, center_circle, radius_circle, Scalar::all(255), -1, 8, 0);
            
        }
        
    //    /* 检测圆形轮廓
    //     第五个参数为能检测到的圆的圆心之间的最小距离
    //     第六个参数为传递给canny边缘检测的高阈值，而低阈值为高阈值的一半
    //     第七个参数越小，就可以检测到更多根本不存在的圆。它越大，能通过检测的圆就更接近圆形
    //     第八个参数为圆半径的最小值
    //     第九个参数为圆半径的最大值*/
    //    vector<Vec3f> circles;
    //    HoughCircles(contour, circles, HOUGH_GRADIENT, 1, src.rows / 10, 200, 20.5, 0, src.rows);
    //
    //    for(size_t i = 0; i < circles.size(); i++)
    //    {
    //        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    //        int radius = cvRound(circles[i][2]);
    //
    //        cv::circle(circle, center, 3, Scalar(0,255,0), -1, 8, 0);
    //        cv::circle(circle, center, radius, Scalar(155,50,255), 3, 8, 0);
    //    }
    

    

    #ifdef DEBUG
    imshow("contours_all Image", allContour);
    imshow("contours_filtered Image", contour);
    imshow("circle Image", circle);
    imshow("craters", craters_);
    waitKey(30);
    #endif
    
//    cout << "craters:" << mask_crater << endl;

}

void MoonLanding::detectRocks()
{
    Mat gray, mask_light, mask_shadow, mask_rock;
    
    mask_light = Mat::zeros(img_map_gray_.rows, img_map_gray_.cols, CV_8UC1);
    mask_shadow = Mat::zeros(img_map_gray_.rows, img_map_gray_.cols, CV_8UC1);
    mask_rock = Mat::zeros(img_map_gray_.rows, img_map_gray_.cols, CV_8UC1);
        
    // 限制对比度的自适应直方图均衡化（CLAHE）。观感较好的图片不需要此操作
    double clipLimit = 1.0;
    Size gridSize= Size(7,7);
    Ptr<cv::CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(clipLimit);
    clahe->setTilesGridSize(gridSize);
    clahe->apply(img_map_gray_, gray);
    
    threshold(gray, mask_light, 190, 255, THRESH_BINARY);
    threshold(gray, mask_shadow, 60, 255, THRESH_BINARY_INV);
//    adaptiveThreshold(gray, mask_light, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 255, 0);
//    adaptiveThreshold(gray, mask_shadow, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 255, 0);
    rocks_ = mask_light + mask_shadow;
//    addWeighted(mask_light, 1, mask_shadow, 1, 0.0, mask_rock);
    
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(rocks_, rocks_, MORPH_OPEN, element);
    morphologyEx(rocks_, rocks_, MORPH_CLOSE, element);
    
    
//    imshow("CLAHE gray", gray);
//    imshow("mask_light", mask_light);
//    imshow("mask_shadow", mask_shadow);
    

    #ifdef DEBUG
    imshow("rocks", rocks_);
    waitKey(30);
    #endif
    
//    cout << "rocks:" << mask_rock << endl;
    
//    return mask_rock;

}

void MoonLanding::searchLandArea()
{
    img_map_ = img_sub_.clone();
    img_map_to_draw_ = img_map_.clone();
    cvtColor(img_map_, img_map_gray_, COLOR_BGR2GRAY);
    
    craters_ = Mat::zeros(img_map_gray_.rows, img_map_gray_.cols, CV_8UC1);
    rocks_ = Mat::zeros(img_map_gray_.rows, img_map_gray_.cols, CV_8UC1);
    obstacles_ = Mat::zeros(img_map_gray_.rows, img_map_gray_.cols, CV_8UC1);
    
    detect_and_compute(img_map_gray_, kpts_map_, desc_map_);
    
    camera_pose_last_ = Point3f(img_map_.cols / 2, img_map_.rows / 2, 0);


    detectCraters();
    #ifdef DEBUG
    ROS_INFO("after detectCraters");
    #endif
    detectRocks();
    #ifdef DEBUG
    ROS_INFO("after detectRocks");
    #endif
    
    obstacles_ = craters_ + rocks_;

    #ifdef DEBUG
    imshow("obstacles", obstacles_);
    waitKey(30);
    #endif

    // 基于膨胀的方法
    Mat obs_dilate, riskMap;
    Size areaSize(obstacles_.rows / 10, obstacles_.rows / 10);  // 根据无人机高度调整
    Mat element = getStructuringElement(MORPH_ELLIPSE, areaSize);
    morphologyEx(obstacles_, obs_dilate, MORPH_DILATE, element);

    #ifdef DEBUG
    ROS_INFO("dilating...");
    #endif

    
    Size selectSize(obstacles_.rows / 10, obstacles_.rows / 10);  // 根据无人机高度调整
    Mat areaKernel = Mat::ones(selectSize, CV_32FC1) / 255;
    filter2D(obs_dilate, riskMap, CV_16UC1, areaKernel);

    #ifdef DEBUG
    ROS_INFO("filtering...");
    #endif
    
    Mat mask_oneThird, mask_twoThird;
    mask_oneThird = Mat::zeros(riskMap.rows, riskMap.cols, CV_8UC1);
    mask_twoThird = Mat::zeros(riskMap.rows, riskMap.cols, CV_8UC1);
    
    mask_oneThird(Rect(riskMap.cols/3,riskMap.rows/3, riskMap.cols/3, riskMap.rows/3)).setTo(255);
    mask_twoThird(Rect(riskMap.cols/6,riskMap.rows/6, riskMap.cols*2/3, riskMap.rows*2/3)).setTo(255);

    #ifdef DEBUG
    ROS_INFO("creating mask...");
    #endif
    
    double minVal_oneThird, minVal_twoThird, minVal_global, minVal;
    Point minLoc_oneThird, minLoc_twoThird, minLoc_global, minLoc;
    
    minMaxLoc(riskMap, &minVal_oneThird, NULL, &minLoc_oneThird, NULL, mask_oneThird);
    cout << "minLoc_oneThird: " << minLoc_oneThird << "    minVal_oneThird:" << minVal_oneThird << endl;
    minMaxLoc(riskMap, &minVal_twoThird, NULL, &minLoc_twoThird, NULL, mask_twoThird);
    cout << "minLoc_twoThird: " << minLoc_twoThird << "    minVal_twoThird:" << minVal_twoThird << endl;
    minMaxLoc(riskMap, &minVal_global, NULL, &minLoc_global, NULL);
    cout << "minLoc_global: " << minLoc_global << "    minVal_global:" << minVal_global << endl;
    
    minVal = min(min(minVal_oneThird, minVal_twoThird), minVal_global);
    if(minVal == minVal_oneThird) minLoc = minLoc_oneThird;
    else if(minVal == minVal_twoThird) minLoc = minLoc_twoThird;
    else if(minVal == minVal_global) minLoc = minLoc_global;

    
    cout << "minLoc: " << minLoc << endl;

    landArea_.center = minLoc;
    landArea_.size = areaSize;
    landArea_.angle = 0;
    
    // 绘制旋转矩形RotatedRect
        Point2f vertices[4];
        landArea_.points(vertices);
        for (int i = 0; i < 4; i++)
            line(img_map_to_draw_, vertices[i], vertices[(i+1)%4], Scalar(0,255,0), 2);
    
    #ifdef DEBUG
    imshow("obs_dilate", obs_dilate);
    imshow("riskMap", riskMap);
    #endif

}


bool MoonLanding::trackingLandmark()
{
    img_camera_ = img_sub_.clone();
    
    double timestamp, timestamp2;
    if(img_camera_.empty()) return false;

    if (img_camera_.channels() != 1) {
        cvtColor(img_camera_, img_camera_gray_, cv::COLOR_RGB2GRAY);
    }
    else
        img_camera_gray_ = img_camera_;


    timestamp = static_cast<double>(clock()) / CLOCKS_PER_SEC;
    detect_and_compute(img_camera_gray_, kpts_camera_, desc_camera_);
    timestamp2 = static_cast<double>(clock()) / CLOCKS_PER_SEC;
    cout << "compute descriptor cost time: " << timestamp2 - timestamp << " s" << endl;

    timestamp = static_cast<double>(clock()) / CLOCKS_PER_SEC;
    match(desc_map_, desc_camera_, matches_);
    timestamp2 = static_cast<double>(clock()) / CLOCKS_PER_SEC;
    cout << "make the match cost time: " << timestamp2 - timestamp << " s" << endl;

    match_mask_.resize(matches_.size(), 1);
    findKeyPointsHomography();

    int goodMatchNum = 0;
    for(int i = 0; i < match_mask_.size(); i++)
    {
        if(match_mask_[i]) goodMatchNum++;
    }

    cout << "质量好的匹配点数量为：" << goodMatchNum << endl;
    // 如果匹配点过少，则重新拍摄一张图片作为新的地图，在新地图的基础上进行匹配
    if(goodMatchNum < 10)
    {
        kSegNum++;
        if(kSegNum > 5) return false;
        cout << "匹配点太少，将开始第" << kSegNum+1 << "段图像导航" << endl;
        
        searchLandArea();

        timestamp = static_cast<double>(clock()) / CLOCKS_PER_SEC;
        match(desc_map_, desc_camera_, matches_);
        timestamp2 = static_cast<double>(clock()) / CLOCKS_PER_SEC;
        cout << "make the match cost time: " << timestamp2 - timestamp << " s" << endl;
    }



    Mat res;
    cv::drawMatches(img_map_to_draw_, kpts_map_, img_camera_, kpts_camera_, matches_, res, Scalar::all(-1),
        Scalar::all(-1), match_mask_, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    
    
    timestamp = static_cast<double>(clock()) / CLOCKS_PER_SEC;
    calcAveragePts();
    timestamp2 = static_cast<double>(clock()) / CLOCKS_PER_SEC;
    cout << "calculate average point cost time: " << timestamp2 - timestamp << " s" << endl;
    
    
    timestamp = static_cast<double>(clock()) / CLOCKS_PER_SEC;
    calcScale();
    timestamp2 = static_cast<double>(clock()) / CLOCKS_PER_SEC;
    cout << "calculate scale cost time: " << timestamp2 - timestamp << " s" << endl;
    

    timestamp = static_cast<double>(clock()) / CLOCKS_PER_SEC;
    calcAngle();
    timestamp2 = static_cast<double>(clock()) / CLOCKS_PER_SEC;
    cout << "calculate angle cost time: " << timestamp2 - timestamp << " s" << endl;
//    cout << "camera angle: " << camera_angle << endl;
    

    camera_position_rotated_ = (Point2f(img_camera_gray_.cols/2, img_camera_gray_.rows/2) - average_pts_[1]) * scale_ + average_pts_[0];
    
    camera_position_.x = (camera_position_rotated_.x - average_pts_[0].x) * cos(angle_ * M_PI / 180) - (camera_position_rotated_.y - average_pts_[0].y) * sin(-angle_ * M_PI / 180) + average_pts_[0].x;
    camera_position_.y = (camera_position_rotated_.x - average_pts_[0].x) * sin(-angle_ * M_PI / 180) + (camera_position_rotated_.y - average_pts_[0].y) * cos(angle_ * M_PI / 180) + average_pts_[0].y;
    
    Point3f current_pose;
    current_pose.x = camera_position_.x;
    current_pose.y = camera_position_.y;
    current_pose.z = angle_;
    
    // 如果识别到的位姿与上一帧位姿相差太大，这一帧会被丢弃．错误次数超过阈值则返回错误
    if((abs(current_pose.x - camera_pose_last_.x) > (img_camera_gray_.cols / 8)) ||  
      (abs(current_pose.y - camera_pose_last_.y) > (img_camera_gray_.rows / 8)) || (abs(current_pose.z - camera_pose_last_.z) > 30))
    {
        kWrongPoseNum_++;
        if(kWrongPoseNum_ <= 10)
        {
            current_pose_ = camera_pose_last_;
            cout << "get a wrong pose!!  Total num: " << kWrongPoseNum_ << endl;
            return true;
        }
        else return false;
    }
    kWrongPoseNum_ = 0;
    current_pose_ = current_pose;
    
    Point2f angle_marker, angle_mark(camera_position_.x, camera_position_.y - 30);
    angle_marker.x = (angle_mark.x - camera_position_.x) * cos(angle_ * M_PI / 180) - (angle_mark.y - camera_position_.y) * sin(-angle_ * M_PI / 180) + camera_position_.x;
    angle_marker.y = (angle_mark.x - camera_position_.x) * sin(-angle_ * M_PI / 180) + (angle_mark.y - camera_position_.y) * cos(angle_ * M_PI / 180) + camera_position_.y;
    
    
//    circle(map_img, average_pts[0], 7, Scalar(0, 0, 255), -1);
    circle(img_map_to_draw_, camera_position_, 5, Scalar(0, 255, 0), -1);
//    circle(map_img, camera_position, 2, Scalar::all(255), -1);
    // circle(img_map_to_draw_, angle_marker, 4, Scalar::all(255), -1);
//    circle(map_img, angle_mark, 4, Scalar::all(255), -1);
    // line(img_map_to_draw_, camera_position_, angle_marker, Scalar::all(255), 2);
    line(img_map_to_draw_, camera_position_, Point2f(camera_pose_last_.x, camera_pose_last_.y), Scalar(0, 255, 0), 2);
//    line(map_img, camera_position_rotated, average_pts[0], Scalar(0, 0, 255), 3);
    
    
//    circle(camera_img, average_pts[1], 7, Scalar(0, 0, 255), -1);
    circle(img_camera_, Point(img_camera_.cols/2, img_camera_.rows/2), 7, Scalar(0, 255, 0), -1);
//    line(camera_img, Point(img2.cols/2, img2.rows/2), average_pts[1], Scalar(0, 0, 255), 3);
    

    #ifdef DEBUG
    cv::imshow("match result", res);
    cv::imshow("map_img_drawed", img_map_to_draw_);
    cv::imshow("camera_img", img_camera_);
    waitKey(10);
    #endif

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", res).toImageMsg();
    match_img_pub_.publish(msg);

    
    camera_pose_last_ = current_pose_;

    return true;
}

void MoonLanding::detect_and_compute(Mat& img, vector<KeyPoint>& kpts, Mat& desc) {
    if (desc_type_.find("fast") == 0) {
        desc_type_ = desc_type_.substr(4);
        Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(10, true);
        detector->detect(img, kpts);
    }
    if (desc_type_.find("blob") == 0) {
        desc_type_ = desc_type_.substr(4);
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create();
        detector->detect(img, kpts);
    }
    if (desc_type_ == "surf") {
        Ptr<Feature2D> surf = SURF::create(50.0, 6, 3, true); // 参数可调整
        surf->detectAndCompute(img, Mat(), kpts, desc);
    }
    if (desc_type_ == "sift") {
        Ptr<Feature2D> sift = SIFT::create();
        sift->detectAndCompute(img, Mat(), kpts, desc);
    }
    if (desc_type_ == "orb") {
        Ptr<ORB> orb = ORB::create();
        orb->detectAndCompute(img, Mat(), kpts, desc);
    }
    if (desc_type_ == "brisk") {
        Ptr<BRISK> brisk = BRISK::create();
        brisk->detectAndCompute(img, Mat(), kpts, desc);
    }
    if (desc_type_ == "kaze") {
        Ptr<KAZE> kaze = KAZE::create();
        kaze->detectAndCompute(img, Mat(), kpts, desc);
    }
    if (desc_type_ == "akaze") {
        Ptr<AKAZE> akaze = AKAZE::create();
        akaze->detectAndCompute(img, Mat(), kpts, desc);
    }
    if (desc_type_ == "freak") {
        Ptr<FREAK> freak = FREAK::create();
        freak->compute(img, kpts, desc);
    }
    if (desc_type_ == "daisy") {
        Ptr<DAISY> daisy = DAISY::create();
        daisy->compute(img, kpts, desc);
    }
    if (desc_type_ == "brief") {
        Ptr<BriefDescriptorExtractor> brief = BriefDescriptorExtractor::create(64);
        brief->compute(img, kpts, desc);
    }
}


void MoonLanding::match(Mat& desc1, Mat& desc2, vector<DMatch>& matches) {
    matches.clear();
    if (match_type_ == "bf") {
        BFMatcher desc_matcher(cv::NORM_L2, true);
        desc_matcher.match(desc1, desc2, matches, Mat());
    }
    if (match_type_ == "knn") {
        BFMatcher desc_matcher(cv::NORM_L2, true);
        vector< vector<DMatch> > vmatches;
        desc_matcher.knnMatch(desc1, desc2, vmatches, 1);
        for (int i = 0; i < static_cast<int>(vmatches.size()); ++i) {
            if (!vmatches[i].size()) {
                continue;
            }
            matches.push_back(vmatches[i][0]);
        }
    }
    std::sort(matches.begin(), matches.end());
    while (max(matches.front().distance * kDistanceCoef_, kMinDistanceThreshold_) < matches.back().distance) {
        matches.pop_back();
    }
    while (matches.size() > kMaxMatchingSize_) {
        matches.pop_back();
    }
}


void MoonLanding::findKeyPointsHomography()
{
    match_points_map_.clear();
    match_points_camera_.clear();
    if (static_cast<int>(match_mask_.size()) < 3) {
        return;
    }
    
    for (int i = 0; i < static_cast<int>(matches_.size()); ++i) {
        match_points_map_.push_back(kpts_map_[matches_[i].queryIdx].pt);
        match_points_camera_.push_back(kpts_camera_[matches_[i].trainIdx].pt);
    }
    findHomography(match_points_map_, match_points_camera_, cv::RANSAC, 4, match_mask_);
}

void MoonLanding::calcAveragePts()
{
    average_pts_.clear();
    
    float x_sum1 = 0, y_sum1 = 0, x_sum2 = 0, y_sum2 = 0;
    Point2f average_pts1, average_pts2;
    int count = 0;
    for(int i = 0; i < matches_.size(); ++i)
    {
        if(match_mask_[i])
        {
            x_sum1 += match_points_map_[i].x;
            y_sum1 += match_points_map_[i].y;
            
            x_sum2 += match_points_camera_[i].x;
            y_sum2 += match_points_camera_[i].y;
            count++;
        }
    }
    average_pts1.x = x_sum1 / count;
    average_pts1.y = y_sum1 / count;
    average_pts_.push_back(average_pts1);
    
    average_pts2.x = x_sum2 / count;
    average_pts2.y = y_sum2 / count;
    average_pts_.push_back(average_pts2);
   
}

void MoonLanding::calcScale()
{
    double sum = 0;
    int count = 0;
    for(int i = 0; i < matches_.size(); ++i)
    {
        if(match_mask_[i])
        {
            sum += (pow(match_points_map_[i].x - average_pts_[0].x, 2) + pow(match_points_map_[i].y - average_pts_[0].y, 2)) / (pow(match_points_camera_[i].x - average_pts_[1].x, 2)  + pow(match_points_camera_[i].y - average_pts_[1].y, 2));
            count++;
        }
    }
    
    scale_ = sqrt(sum / count);

}


void MoonLanding::calcAngle()
{
    double sum = 0;
    int count = 0;
    for(int i = 0; i < matches_.size() - 1; ++i)
    {
        if(match_mask_[i])
        {
            sum += (atan2((match_points_map_[i].x - average_pts_[0].x), (match_points_map_[i].y - average_pts_[0].y)) - atan2((match_points_camera_[i].x - average_pts_[1].x), (match_points_camera_[i].y - average_pts_[1].y)));
            count++;
        }
    }

    angle_ = (sum / count) * 180 / M_PI;

}


void MoonLanding::AdaptiveFindThreshold(cv::InputArray src, double *low, double *high, int aperture_size)
{
    const int cn = src.channels();
    cv::Mat dx(src.rows(), src.cols(), CV_16SC(cn));
    cv::Mat dy(src.rows(), src.cols(), CV_16SC(cn));
                                                                               
    cv::Sobel(src, dx, CV_16S, 1, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE);
    cv::Sobel(src, dy, CV_16S, 0, 1, aperture_size, 1, 0, cv::BORDER_REPLICATE);
                                                                               
    Mat _dx = dx, _dy = dy;
    _AdaptiveFindThreshold(&_dx, &_dy, low, high);
                                                                               
}
                                                                               
// 仿照matlab，自适应求高低两个门限
void MoonLanding::_AdaptiveFindThreshold(Mat *dx, Mat *dy, double *low, double *high)
{
    Size size;
    int i,j;
    MatND hist;
    int hist_size = 255;
    float range_0[]={0,256};
    const float* ranges[] = { range_0 };
    double PercentOfPixelsNotEdges = 0.7;
    size = dx->size();
    Mat imge(size, CV_32FC1);
    // 计算边缘的强度, 并存于图像中
    float maxv = 0;
    for(i = 0; i < size.height; i++ )
    {
        const short* _dx = (short*)(dx->data + dx->step*i);
        const short* _dy = (short*)(dy->data + dy->step*i);
        float* _image = (float *)(imge.data + imge.step*i);
        for(j = 0; j < size.width; j++)
        {
            _image[j] = (float)(abs(_dx[j]) + abs(_dy[j]));
            maxv = maxv < _image[j] ? _image[j]: maxv;
                                                                           
        }
    }
    if(maxv == 0){
        *high = 0;
        *low = 0;
        return;
    }
                                                                               
    // 计算直方图
    range_0[1] = maxv;
    hist_size = (int)(hist_size > maxv ? maxv:hist_size);
    int channels = 0;
    cv::calcHist(&imge, 1, &channels, Mat(), hist, 1, &hist_size, ranges);
    int total = (int)(size.height * size.width * PercentOfPixelsNotEdges);
    float sum=0;
    int icount = hist.dims;
                                                                               
    float *h = (float*)hist.data;
    for(i = 0; i < icount; i++)
    {
        sum += h[i];
        if( sum > total )
            break;
    }
    // 计算高低门限
    *high = (i+1) * maxv / hist_size ;
    *low = *high * 0.4;
}


