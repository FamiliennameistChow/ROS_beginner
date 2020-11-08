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


//-----------------------------------【main( )函数】--------------------------------------------
//		描述：控制台应用程序的入口函数，我们的程序从这里开始
//-----------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "find_circle");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate rate_ = ros::Rate(20.0);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_depth_sub = it.subscribe("/kinect/depth/image_raw", 1, imageDepthCallback); //订阅深度图
    image_transport::Subscriber img_front_sub = it.subscribe("/kinect/rgb/image_raw", 1, imageFrontCallback); //订阅前置摄像头
    image_transport::Subscriber img_bottom_sub = it.subscribe("/camera/depth/image_raw", 1, imageBottomCallback); //订阅下置摄像头

    image_transport::Publisher pub = it.advertise("find_circle/image", 1);


    ros::Subscriber darknet_bounding_box_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 10, darknetBBCallback);

    ros::Publisher front_camera_reslut_pub = nh.advertise<geometry_msgs::Point>("find_circle/reslut_in_dody", 10);

    nh_.param<int>("finding_method", finding_method, 1);
    
    while(img_depth.empty())
    {
        cout<<"no image data!!"<<endl;
        ros::spinOnce();
        rate_.sleep();
    }


    while(ros::ok())
    {
        if (finding_method == 0 || finding_method == 2)
        {
            center_point_in_camrea.z = 99;
            cout << "[find circle] use the method of depth...." << endl;
            Mat binaryImage;//临时变量和目标图的定义
            Mat dstImage = Mat::zeros(img_depth.rows, img_depth.cols, CV_8UC1);

            //【3】转为灰度图并进行图像平滑
            //cvtColor(img_sub, grayImage, COLOR_BGR2GRAY);//转化边缘检测后的图为灰度图
            //GaussianBlur(img_sub, grayImage, Size(9, 9), 2, 2 );
            inRange(img_depth, 0.3, 3, binaryImage);

            //imshow("binaryImage", binaryImage);
            //waitKey(10);

            //【4】进行霍夫圆变换
            vector<Vec3f> circles;
            HoughCircles(binaryImage, circles, CV_HOUGH_GRADIENT, 1, 10, 200, 35, 0, 0 );

            //【5】依次在图中绘制出圆
            Point center_in_pixel;
            int radius;
            cout << "[find circle] size: " << circles.size() << endl;
            if (circles.size() == 0)
            {
                continue;
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

            //绘制圆心
            circle(img_front, center_in_pixel, 3, Scalar(255, 0, 0), -1, 8, 0 );
            //绘制圆轮廓
            circle(img_front, center_in_pixel, radius, Scalar(0 , 255, 255), 3);

            Point p_tl;
            p_tl.x = min(max(center_in_pixel.x - radius - 10, 0), img_depth.cols);
            p_tl.y = min(max(center_in_pixel.y - radius - 10, 0), img_depth.rows);

            Rect roi(p_tl.x, p_tl.y, radius*2+20, radius*2+20);
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
            
            cout << "count :" << count << " " << value_sum << endl;
            cout << "center pose:  " << center_in_pixel.x << " " << center_in_pixel.y << " " << center_point_in_camrea.z << " " << radius<< endl;
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

            // //【6】显示效果图  
            imshow("process", img_front);
            waitKey(1); 
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_front).toImageMsg();
            pub.publish(msg); 
        }
        
        
        if (finding_method == 1 || finding_method == 2)
        {
            cout << "[find circle] use the method of darknet...." << endl;
            center_point_in_camrea.z = 99;
            if(BBList.empty()){
                ros::spinOnce();
                rate_.sleep();
                continue;
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
            imshow("process", img_front);
            waitKey(1);
        } //end esle if
        
    
        ros::spinOnce();
        rate_.sleep();
    }
        

        return 0;  
}