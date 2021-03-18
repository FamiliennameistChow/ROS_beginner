#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/ml/ml.hpp>
#include <opencv2/ml.hpp>

#include <riverdetect/centerpoints.h>

#include <iostream>
#include <math.h>
#include <string.h>
#include <unistd.h>

using namespace cv;
using namespace std;
using namespace cv::ml;


//全局变量Mat
riverdetect::centerpoints point_result;
Mat  img_sub;
geometry_msgs::Point point_center;
float theta = 0.;
float error_x = 0.;
int num = 0;


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

Mat get_perspective_img(Mat& image, int px1, int px2, int px3, int px4)
{
    resize(image, image, Size(1280, 720));
    int wrap_offset = 50;
    cv::Point2f src_points[] = {
        cv::Point2f(px1, 0),
        cv::Point2f(px2, 0),
        cv::Point2f(px3, 360),
        cv::Point2f(px4, 360) };

    cv::Point2f dst_points[] = {
        cv::Point2f(px1, 0),
        cv::Point2f(px2, 0),
        cv::Point2f(px1, image.rows/2),
        cv::Point2f(px2, image.rows/2) };

    Mat M = cv::getPerspectiveTransform(src_points, dst_points);

    Mat perspective;
    warpPerspective(image, perspective, M, cv::Size(image.cols, image.rows), cv::INTER_LINEAR);
    return perspective;
}


void drawLine(Mat& image, Vec4f& line, bool cal_error)
{
    Point point1, point2, point3;
    point1.x = line[2];
    point1.y = line[3];

    float k = line[1] / line[0];

    point2.x = point1.x - point1.y/k;
    point2.y = 0;

    point3.y = image.rows;
    point3.x = (point3.y - point1.y)/k + point1.x;


    cv::line(image, point2, point3, cv::Scalar(0, 0, 255), 2);
    circle(image, Point(image.cols/2, image.rows/2), 3, Scalar(0, 255, 255), 1);

    if(cal_error) // 计算角度
    {
        theta = atan2(point3.x - point2.x, point3.y - point2.y);
//        theta = theta * 180.0 / CV_PI;
         cout << "angle:" << theta << endl;
        error_x = (point2.x + point3.x) /2 - image.cols/2;
        cout << "err_x :" << error_x << endl;
    }

}

void drawLineSplit(Mat& image, Vec4f& line, int start, bool cal_error)
{
    Point point1, point2, point3;
    int distant;
    point1.x = line[2];
    point1.y = line[3];

    float k = line[1] / line[0];

    point2.y = start;
    point2.x = point1.x + (point2.y - point1.y)/k;


    point3.y = start + image.rows/10;
    point3.x = (point3.y - point1.y)/k + point1.x;
    
    //设定阈值，若检测差太多则不画线
    distant = abs(point2.x -point3.x);//改点

    if (distant < image.cols/7){
    cv::line(image, point2, point3, cv::Scalar(255, 0, 255), 2);
    circle(image, Point(image.cols/2, image.rows/2), 3, Scalar(0, 255, 255), 1);

     if(cal_error) // 计算角度
    {
        theta += atan2(point3.x - point2.x, point3.y - point2.y);
//        theta = theta * 180.0 / CV_PI;
        cout << "angle:" << theta << endl;
        error_x += (point2.x + point3.x) /2 - image.cols/2;
        num++;
        cout << "err_x :" << error_x << endl;
    }
    }
}

void detect(Mat& img)
{
     Mat img_hsv, img_gray;
//    img = imread("../12.jpg");
    resize(img, img, Size(1280, 720));
    imshow("img", img);
    cvtColor(img, img_hsv, COLOR_BGR2HSV);
    cvtColor(img, img_gray, COLOR_BGR2GRAY);

    vector<Mat> hsv;
    split(img_hsv, hsv);
    imshow("h", hsv[0]);
    imshow("s", hsv[1]);
    imshow("v", hsv[2]);

    Mat img_blur;
    int kernel_bulr = 5;
    medianBlur(hsv[1], img_blur, kernel_bulr);
    imshow("img_blur", img_blur);
//    threshold(img_blur, img_blur, 50, 255, THRESH_BINARY);
//    imshow("img_blur thre", img_blur);
//    waitKey(0);

    Mat img_x(img.size(), CV_16SC1);
    Mat img_y(img.size(), CV_16SC1);
    Mat img_x_abs(img.size(), CV_8UC1);
    Mat img_y_abs(img.size(), CV_8UC1);
    Sobel(img_blur, img_x, CV_16SC1, 1, 0, 3);
//    medianBlur(img_x, img_x, 3);
    Sobel(img_blur, img_y, CV_16SC1, 0, 1, 3);
//    medianBlur(img_y, img_y, 3);
    convertScaleAbs(img_x, img_x_abs);
    convertScaleAbs(img_y, img_y_abs);
    imshow("img_x", img_x_abs);
    imshow("img_y", img_y_abs);
    Mat dst;
    addWeighted(img_x_abs, 0.8, img_y_abs, 0.2, 0, dst);
//    medianBlur(dst, dst, 3);
    imshow("dst1", dst);
    threshold(dst, dst, 50, 255, THRESH_BINARY);
    imshow("count image", dst);


    vector<Point> left_points;
    vector<Point> right_points;
    vector<Point> center_points;
    int rows = dst.rows;
    int cols = dst.cols;

//**********透视**********
//    int left_x1, right_x1, left_x2, right_x2;

//    const uchar* row1 = dst.ptr<uchar>(0);
//    for(int i=0; i<cols/2; i++)
//    {
//        if(row1[i] == 255)
//        {
//            left_x1 = i;
//        }
//    }

//    for(int j=cols/2; j<cols; j++)
//    {
//        if(row1[j] == 255)
//        {
//            right_x1 = j;
//        }
//    }

//    const uchar* row2 = dst.ptr<uchar>(rows/2);
//    for(int i=0; i<cols/2; i++)
//    {
//        if(row2[i] == 255)
//        {
//            left_x2 = i;
//        }
//    }

//    for(int j=cols/2; j<cols; j++)
//    {
//        if(row2[j] == 255)
//        {
//            right_x2 = j;
//        }
//    }

//    dst = get_perspective_img(dst, left_x1, right_x1, left_x2, right_x2);
//    imshow("after perspective image", dst);
//************************

    int left_x, right_x, center_x, left_x_last, right_x_last;
    Mat point_img;
    point_img.create(img.size(),img.type());
    for(int k=0; k< rows; k++)
    {
        const uchar* row = dst.ptr<uchar>(k);
        for(int i=0; i<cols/2; i++)
        {
//            if(row[i] == 255)
//            {
//                if(k == 0)
//                {
//                    left_x = i;
//                }else{
//                    if(abs(left_x - i) < 100)
//                    {
//                        left_x = i;
//                    }
//                }
//            }
            if(row[i] == 255)
            {
                left_x = i;
            }
        }

        for(int j=cols/2; j<cols; j++)
        {
            if(row[j] == 255)
            {

//                if(k == 0)
//                {
//                    right_x = j;
//                }else{
//                    if(abs(right_x - j) < 100)
//                    {
//                        right_x = j;
//                    }
//                }
                right_x = j;
                break;
            }
        }


        center_x = (left_x + right_x) / 2;

//        circle(img, Point(center_x, k), 2, Scalar(0, 0, 255), -1);
//        circle(img, Point(left_x, k), 2, Scalar(255, 255, 0));
//        circle(img, Point(right_x, k), 2, Scalar(0, 255, 255));
        left_points.push_back(Point(left_x, k));
        right_points.push_back(Point(right_x, k));
        center_points.push_back(Point(center_x, k));
        point_center.x = center_x;
        point_center.y = k;
        point_result.rd_point.push_back(point_center);
    }


// *********分段拟合**************
   int start = 0;
    Vec4f split_line;
    vector<Point> choosed_point;
    Point choosed_tl_point, choosed_tl_point_left, choosed_tl_point_right;
    for(int i = 0; i< 10; i++)
    {
        choosed_point.clear();
        for(int j=start; j<start+center_points.size()/10; j++)
        {
            //窗口显示
            if (j == start)
            {
                choosed_tl_point = Point(center_points.at(j).x - 50, center_points.at(j).y);
                choosed_tl_point_left = Point(left_points.at(j).x - 50, left_points.at(j).y);
                choosed_tl_point_right = Point(right_points.at(j).x - 50, right_points.at(j).y);
                Rect rect_center(choosed_tl_point.x, choosed_tl_point.y, 100, center_points.size()/10);
                Rect rect_left(choosed_tl_point_left.x, choosed_tl_point_left.y, 100, center_points.size()/10);
                Rect rect_right(choosed_tl_point_right.x, choosed_tl_point_right.y, 100, center_points.size()/10);
                cv::rectangle(img, rect_center, Scalar(100, 100, 100), 1, LINE_8,0);
                cv::rectangle(img, rect_left, Scalar(255, 0, 0), 1, LINE_8,0);
                cv::rectangle(img, rect_right, Scalar(255, 0, 0), 1, LINE_8,0);
            }
            choosed_point.push_back(center_points.at(j));
        }

        if(i==4 || i==5 || i==6 || i==3){
//            for(auto it = choosed_point.begin(); it != choosed_point.end(); it++)
//            {
//                circle(img, *it, 3, Scalar(255, 100, 200), 1);
//            }image.cols/2
            fitLine(choosed_point, split_line, CV_DIST_L2 ,0, 0.01, 0.01);
//            cout <<"split_line :" <<  split_line << endl;
            drawLineSplit(img, split_line, start, true);
        }
//        fitLine(choosed_point, split_line, CV_DIST_L1,0, 1e-2, 1e-2);
//        cout <<"split_line :" <<  split_line << endl;
//        drawLineSplit(img, split_line, start, true);
        start = start + center_points.size()/10;
    }

    error_x = error_x/num;
    point_result.error = error_x;
    theta = theta/num;
    point_result.angle = theta;
    cout << "====" << endl;
    cout << "angle:" << theta << endl;
    cout << "err_x:" << error_x << endl;
// *****************************
   if (-img.cols/4 < error_x < img.cols/4){
       point_result.centersuccess = true;
   }else{
       point_result.centersuccess = false;
   }

    Vec4f left_line, right_line, center_line;
    fitLine(left_points,left_line, CV_DIST_L1,0,0.01,0.01);
    fitLine(right_points,right_line, CV_DIST_L1,0,0.01,0.01);
//    fitLine(center_points,center_line, CV_DIST_L1,0,0.01,0.01);


//    drawLine(img, left_line, false);
//    drawLine(img, right_line, false);
//    drawLine(img, center_line, true);


    imshow("point image", img);
//    imshow("point", point_img);
}

int main(int argc, char** argv)
{
//    //打开一个视频
//    VideoCapture cap("/media/bornchow/FILE/program_working/waterQualityPlatform/data_river/DJI_0768.MOV");
//    if (!cap.isOpened()) {
//        cerr << " can not open a camera or file" << endl;
//        return -1;
//    }

//    while(1)
//    {
//        Mat frame;
//        //从cap中读出一帧  存放在frame
//        cap >> frame;

//        //如果未读到图像
//        if (frame.empty()) {
//            break;
//        }

//        detect(frame);
//    }

    cout << "Using OpenCV version\n" << CV_VERSION << "\n" << endl;
    // getPath();
    ros::init(argc, argv, "centerpoints_detect");
    ros::NodeHandle nh;
    ros::Rate rate_ = ros::Rate(20.0);


  //【发布】处理结果
    ros::Publisher centerpoints_pub = nh.advertise<riverdetect::centerpoints>("river_detect/centerpoints", 100);

    //image_transport 负责图像发布与订阅
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/iris_rplidar/usb_cam/image_raw", 1, imageCallback);
    image_transport::Publisher pub = it.advertise("centerpoint/image/centerpoint", 1);

 while(img_sub.empty())
    {
        cout<<"no image data!!"<<endl;
        ros::spinOnce();
        rate_.sleep();
        continue;
    }

    Mat image;
    // vector<vector<Point> > squares;
    // Mat img;
   // Point center;

    while(ros::ok())
    {    

        img_sub.copyTo(image);
        detect(image);

       // cout << "detect time cost:         " << time_used_total.count()     << endl;

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        pub.publish(msg);

        centerpoints_pub.publish(point_result);

       cout << point_result.centersuccess << endl;
        //imwrite( "out", image );image.cols/2
        int c = waitKey(1);
        if( (char)c == 27 )
            break;
        ros::spinOnce();
        rate_.sleep();
    }

    return 0;
}


