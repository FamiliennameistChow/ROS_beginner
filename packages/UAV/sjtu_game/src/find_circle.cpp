//---------------------------------【头文件、命名空间包含部分】-------------------------------
//          描述：包含程序所使用的头文件和命名空间
//----------------------------------------------------------------------------------------------
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

// darknet 消息
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <map>

using namespace cv;
using namespace std;


Mat img_depth, img_front, img_bottom;
int finding_method;
cv_bridge::CvImagePtr cv_ptr;
map<int, darknet_ros_msgs::BoundingBox> BBList;
map<int, darknet_ros_msgs::BoundingBox>::iterator BBList_iter;
geometry_msgs::Point center_point_in_body, center_point_in_camrea;
geometry_msgs::Point land_center_err_in_pixel;

ros::Publisher front_camera_reslut_pub;
ros::Publisher bottom_camera_reslut_pub;
image_transport::Publisher result_img_pub;

bool landing_mark=false;

float cx = 321.04638671875;
float cy = 243.44969177246094;
float fx = 387.229248046875;
float fy = 387.229248046875;


//-----------------------------------【回调函数】--------------------------------------------
//		描述：回调图像数据与dark_net消息
//-----------------------------------------------------------------------------------------------
void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
    try
    {
    //  cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    //  img_sub = cv_ptr->image;
    cv_ptr->image.copyTo(img_depth);
}

void imageFrontCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // cv_bridge::CvImagePtr cv_ptr;
    try
    {
     cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    //  img_sub = cv_ptr->image;
    cv_ptr->image.copyTo(img_front);
}

void imageBottomCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // cv_bridge::CvImagePtr cv_ptr;
    try
    {
     cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    //  img_sub = cv_ptr->image;
    cv_ptr->image.copyTo(img_bottom);
}

void darknetBBCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
    // cout<< "----" << msg->bounding_boxes[0].Class << endl;
    BBList.clear();
    int area;
    for (auto n : msg->bounding_boxes)
    {
        if (n.probability < 0.9)
        {
            continue;
        }

        area = (n.xmax -n.xmin) * (n.ymax-n.ymin);
        BBList.insert(make_pair(area, n)); 
    }
}

void landingMarkCallback(const std_msgs::Bool::ConstPtr &msg){
    landing_mark = msg->data;
}

// ---------------------------------【 图像处理函数】--------------------------------------------
//
//--------------------------------------------------------------------------------------------

void findCircleFromDepth(){

    center_point_in_camrea.z = 99;
    cout << "[find circle] use the method of depth...." << endl;
    Mat binaryImage;//临时变量和目标图的定义

    imshow("img_depth", img_depth);
    waitKey(1);
    Mat dstImage = Mat::zeros(img_depth.rows, img_depth.cols, CV_8UC1);

    //【3】转为灰度图并进行图像平滑
    //cvtColor(img_sub, grayImage, COLOR_BGR2GRAY);//转化边缘检测后的图为灰度图
    //GaussianBlur(img_sub, grayImage, Size(9, 9), 2, 2 );
    inRange(img_depth, 0.3, 3, binaryImage);

    imshow("binaryImage", binaryImage);
    waitKey(1);

    //【4】进行霍夫圆变换
    vector<Vec3f> circles;
    HoughCircles(binaryImage, circles, CV_HOUGH_GRADIENT, 1, 10, 200, 35, 0, 0 );

    //【5】依次在图中绘制出圆
    Point center_in_pixel;
    int radius;
    cout << "[find circle] size: " << circles.size() << endl;
    if (circles.size() == 0)
    {
        return;
    }
    
    for( size_t i = 0; i < circles.size(); i++ )
    {
        //参数定义
        center_in_pixel.x = cvRound(circles[i][0]);
        center_in_pixel.y = cvRound(circles[i][1]);
        // Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        radius = cvRound(circles[i][2]);
        // cout << "center:[" << circles[i][0] << ", " << circles[i][1] << "]   ,  "  << "radius:" << circles[i][2] << endl;
    }
    
    std::cout << "[find circle] radius: " <<  radius << std::endl;
    //绘制圆心
    circle(img_front, center_in_pixel, 3, Scalar(255, 0, 0), -1, 8, 0 );
    //绘制圆轮廓
    circle(img_front, center_in_pixel, radius, Scalar(0 , 255, 255), 3);

    // Point p_tl;
    // p_tl.x = min(max(center_in_pixel.x - radius - 10, 0), img_depth.cols);
    // p_tl.y = min(max(center_in_pixel.y - radius - 10, 0), img_depth.rows);

    Rect roi(center_in_pixel.x - radius - 10, center_in_pixel.y - radius - 10, radius*2+20, radius*2+20);
    if (center_in_pixel.x - radius - 10 < 0 || 
        center_in_pixel.y - radius - 10 < 0 ||
        center_in_pixel.y + radius + 10 > img_depth.rows ||
        center_in_pixel.x + radius + 10 > img_depth.cols)
    {
        std::cout << "[find circle] circle in not in center " << std::endl;
        return;
    }
    

    Mat img_depth_roi = img_depth(roi);

    
    float value;
    float value_sum = 0.0 ;
    int count=0;
    for (int r = 0; r < img_depth_roi.rows; r++)
    {
        for (int c = 0; c < img_depth_roi.cols; c++)
        {
            value = img_depth_roi.at<float>(r, c);
            if (value > 0 && value < 3)
            {
                count++;
                value_sum += value;
            }  
        }  
    }
    center_point_in_camrea.z = value_sum / count;
    
    cout << "[find circle] count :" << count << " " << value_sum << endl;
    cout << "[find circle] center pose:  " << center_in_pixel.x << " " << center_in_pixel.y << " " << center_point_in_camrea.z << " " << radius<< endl;
    // 像素坐标转相机坐标
    // Xc = (u - u0)*Zc / fx
    // 
    center_point_in_camrea.x = (center_in_pixel.x - img_depth.cols/2) * center_point_in_camrea.z / fx;
    center_point_in_camrea.y = (center_in_pixel.y - img_depth.rows/2) * center_point_in_camrea.z / fy;

    // 相机坐标转机体坐标
    center_point_in_body.x = center_point_in_camrea.z;
    center_point_in_body.y = -center_point_in_camrea.x;
    center_point_in_body.z = -center_point_in_camrea.y;
    cout << "[find circle] reslut: " << center_point_in_body.x <<" " <<center_point_in_body.y << " " << center_point_in_body.z << endl;
    
    front_camera_reslut_pub.publish(center_point_in_body);

    // // //【6】显示效果图  
    imshow("process", img_front);
    waitKey(1); 
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_front).toImageMsg();
    result_img_pub.publish(msg); 
}


void findCircleFromYOLO(){
    cout << "[find circle] use the method of darknet...." << endl;
    center_point_in_camrea.z = 99;
    if(BBList.empty()){
        return;
    }

    // for (auto bb : BBList)
    // {
    //     cout <<" area: "<<  bb.first << " " << bb.second.xmin << endl;
    // }

    BBList_iter = BBList.end(); BBList_iter--;

    // cout<<"process::" << BBList_iter->second.xmin << " " << BBList_iter->second.ymin << " " 
    //     << BBList_iter->second.xmax - BBList_iter->second.xmin << " " 
    //     << BBList_iter->second.ymax - BBList_iter->second.ymin << endl;

    Point center_in_pixel; // 像素坐标
    center_in_pixel.x = (BBList_iter->second.xmin + BBList_iter->second.xmax) / 2;
    center_in_pixel.y = (BBList_iter->second.ymin + BBList_iter->second.ymax) / 2;

    circle(img_front, center_in_pixel, 5, Scalar(0,0,255), 3);
    Rect r(BBList_iter->second.xmin, 
        BBList_iter->second.ymin, 
        BBList_iter->second.xmax - BBList_iter->second.xmin, 
        BBList_iter->second.ymax - BBList_iter->second.ymin);
    rectangle(img_front, r, Scalar(0, 255, 255), 3);

    // 计算z方向的值
    Mat img_depth_roi = img_depth(r);
    float value = 0.0;
    int count = 0;
    float value_sum = 0.0;

    // cout << "size: " << img_depth_roi.cols * img_depth_roi.rows << endl;
    for (int r = 0; r < img_depth_roi.rows; r++)
    {
        for (int c = 0; c < img_depth_roi.cols; c++)
        {
            value = img_depth_roi.at<float>(r, c);
            if (value > 0 && value < 3)
            {
                count++;
                value_sum += value;
            }  
        }  
    }
    center_point_in_camrea.z = value_sum / count;

    // 像素坐标转相机坐标
    // Xc = (u - u0)*Zc / fx
    // 
    center_point_in_camrea.x = (center_in_pixel.x - img_front.cols/2) * center_point_in_camrea.z / fx;
    center_point_in_camrea.y = (center_in_pixel.y - img_front.rows/2) * center_point_in_camrea.z / fy;

    // 相机坐标转机体坐标
    center_point_in_body.x = center_point_in_camrea.z;
    center_point_in_body.y = -center_point_in_camrea.x;
    center_point_in_body.z = -center_point_in_camrea.y;
    cout << "[find circle] reslut: " << center_point_in_body.x <<" " <<center_point_in_body.y << " " << center_point_in_body.z << endl;

    front_camera_reslut_pub.publish(center_point_in_body);
    // imshow("process", img_front);
    // waitKey(1);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_front).toImageMsg();
    result_img_pub.publish(msg); 
}


void findLandMark(){
    Mat img_src;
    img_bottom.copyTo(img_src);

    // imshow("img_raw", img_src);
    // waitKey(1);

    Mat img_Luv, L, u, v, u_thre;
    std::vector<Mat> Luv;

    Mat img_HSV, H, S, V, thre;
    std::vector<Mat> HSV;

    Mat img_gray;
    
    cvtColor(img_src, img_Luv, CV_BGR2Luv);
    cvtColor(img_src, img_HSV, CV_BGR2HSV);
    cvtColor(img_src, img_gray, CV_BGR2GRAY);

    split(img_Luv, Luv);
    split(img_HSV, HSV);

    L = Luv[0];
    u = Luv[1];
    v = Luv[2];
    //imshow("L", L);
    //imshow("u", u);
    //imshow("v", v);
    //waitKey(1);

    //cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    //clahe->setClipLimit(2.0);
    //clahe->setTilesGridSize(Size(7, 7));
    //clahe->apply(u, u);
    //imshow("u_after", u);
    //threshold(u, u_thre, 100, 255, CV_THRESH_BINARY_INV);
    //imshow("u_thre", u_thre);
    //waitKey(1);

    H = HSV[0];
    S = HSV[1];
    V = HSV[2];

    // imshow("H", H);
    // imshow("S", S);
    // imshow("V", V);
    // waitKey(1);

    threshold(V, thre, 200, 255, CV_THRESH_BINARY);
    // imshow("S_thre", thre);
    // waitKey(1);

    threshold(img_gray, img_gray, 150, 255, CV_THRESH_BINARY);
    //imshow("img_gray_after thre", img_gray);

    Mat image_and;
    /*cv::bitwise_and(S_thre, u_thre, image_and);*/
    cv::bitwise_and(thre, img_gray, image_and);
    // imshow("image_and", image_and);
    // waitKey(1);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image_and, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    float area;
    float maxArea = 0.0;
    int maxIndex = 0;
    for (int i = 0; i < contours.size(); i++) {
        area = cv::contourArea(contours[i]);
        Rect box = boundingRect(contours[i]);
        if (box.height < box.width*0.8 || box.height > box.width * 1.8)
        {
            return;
        }

        if (area > maxArea)
        {
            maxIndex = i;
            maxArea = area;
        }
    }
    //cv::drawContours(img_src_show, contours, maxIndex, Scalar(255, 0, 255));
    //imshow("img_target", img_src_show);


    Rect landMarkBox = boundingRect(contours[maxIndex]);
    rectangle(img_src, landMarkBox, Scalar(255, 0, 255), 1, 8, 0);

    cv::Point land_center_in_pixel; // 像素坐标
    land_center_in_pixel.x = landMarkBox.x + cvRound(landMarkBox.width/2.0);
    land_center_in_pixel.y = landMarkBox.y + cvRound(landMarkBox.height/2.0);
    circle(img_src, land_center_in_pixel, 5, Scalar(0,0,255), 3);
    circle(img_src, Point(img_src.cols/2, img_src.rows/2), 5, Scalar(255,0,0), 3);

    land_center_err_in_pixel.x = land_center_in_pixel.x - img_src.cols/2;
    land_center_err_in_pixel.y = land_center_in_pixel.y - img_src.rows/2;
    cout << "err: " << land_center_err_in_pixel.x << " " << land_center_err_in_pixel.y << endl;
    bottom_camera_reslut_pub.publish(land_center_err_in_pixel);
    // œµÂäÄ¿±êÏÔÊŸ
    // imshow("img_target", img_src);
    // waitKey(1);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_src).toImageMsg();
    result_img_pub.publish(msg); 

    // Mat img_roi;
    // img_roi = image_and(landMarkBox);

    // threshold(img_roi, img_roi, 100, 255, CV_THRESH_BINARY);
    // //imshow("img_roi_board", img_roi);
    // //waitKey(1);

    // CVContours contours_5;
    // contours_5 = findMaxContours(img_roi, false, true, 5);

    // Rect landMarkNumBox = boundingRect(contours_5[contours_5.size() - 1]);
    // Mat img_num_roi = img_roi(landMarkNumBox);

    // recoNum = numDetector(img_num_roi);

    // if (recoNum == 0)
    // {
    //     cv::Point landCenter;
    //     landCenter.x = landMarkBox.x + cvRound(landMarkBox.width/2.0);
    //     landCenter.y = landMarkBox.y + cvRound(landMarkBox.height/2.0);
    //     result.x = (IMG_WIDTH / 2) - landCenter.x;
    //     result.y = (IMG_HEIGHT / 2) - landCenter.y;
    //     result.z = 0;
    //     return true;

    // }

    // return false;
}

//-----------------------------------【main( )函数】--------------------------------------------
//		描述：控制台应用程序的入口函数，我们的程序从这里开始
//-----------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "find_circle");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate rate_ = ros::Rate(20.0);
    std::string front_rgb_topic, front_depth_topic, bottom_rgb_topic;
    nh_.param<string>("front_rgb_topic",   front_rgb_topic,   "/kinect/rgb/image_raw1");
    nh_.param<string>("front_depth_topic", front_depth_topic, "/kinect/depth/image_raw1");
    nh_.param<string>("bottom_rgb_topic",  bottom_rgb_topic,  "/iris/usb_cam2/image_raw1");
    nh_.param<int>("finding_method",       finding_method,    1);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_depth_sub = it.subscribe(front_depth_topic, 1, imageDepthCallback); //订阅深度图
    image_transport::Subscriber img_front_sub = it.subscribe(front_rgb_topic, 1, imageFrontCallback); //订阅前置摄像头
    image_transport::Subscriber img_bottom_sub = it.subscribe(bottom_rgb_topic, 1, imageBottomCallback); //订阅下置摄像头

    result_img_pub = it.advertise("detect/find_result_image", 1);


    ros::Subscriber darknet_bounding_box_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 10, darknetBBCallback);

    ros::Subscriber landing_mark_sub = nh.subscribe<std_msgs::Bool>("landing", 1, landingMarkCallback);

    front_camera_reslut_pub = nh.advertise<geometry_msgs::Point>("detect/circle_reslut_in_dody", 10);
    bottom_camera_reslut_pub = nh.advertise<geometry_msgs::Point>("detect/land_reslut_in_pixel", 10);

    
    while(img_depth.empty())
    {
        cout<<"no image data!!"<<endl;
        ros::spinOnce();
        rate_.sleep();
    }


    while(ros::ok()) // 逻辑控制
    {
        cout << "if landing: " << landing_mark << endl;
        if (!landing_mark)
        {
            if (finding_method == 0 || finding_method == 2)
            {
                findCircleFromDepth();
            }
            
            if (finding_method == 1 || finding_method == 2)
            {
                findCircleFromYOLO();
            } 
        }
        
        if (landing_mark)
        {
            findLandMark();
        }
        
        ros::spinOnce();
        rate_.sleep();
    }
        

        return 0;  
}