//---------------------------------【头文件、命名空间包含部分】-------------------------------
//          描述：包含程序所使用的头文件和命名空间
//----------------------------------------------------------------------------------------------
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
using namespace cv;
using namespace std;


Mat img_sub;
cv_bridge::CvImagePtr cv_ptr;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {    
    //  cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    //  img_sub = cv_ptr->image;
    cv_ptr->image.copyTo(img_sub);
    //cout << img_sub << endl;
}

//-----------------------------------【main( )函数】--------------------------------------------
//		描述：控制台应用程序的入口函数，我们的程序从这里开始
//-----------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "find_circle");
    ros::NodeHandle nh;
    ros::Rate rate_ = ros::Rate(20.0);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/D435i/depth/image_rect_raw", 1, imageCallback);
    image_transport::Publisher pub = it.advertise("find_circle/result", 1);
    
    while(img_sub.empty())
    {
    cout<<"no image data!!"<<endl;
    ros::spinOnce();
    rate_.sleep();
    }


    while(ros::ok())
    {
        Mat binaryImage, filterImage;//临时变量和目标图的定义
        //Mat dstImage = Mat::zeros(img_sub.rows, img_sub.cols, CV_8UC1);
        
        Mat_<float> imgf_sub = img_sub / 1000; //将深度图数值转换为实际深度（float）
        //cout << imgf_sub << endl;

        //进行图像平滑
        inRange(imgf_sub, 1, 4, binaryImage); // 阈值处理，将1至4米深度内的设为255，增大对比
        GaussianBlur(binaryImage, filterImage, Size(7, 7), 2, 2 ); // 滤波
        //bilateralFilter(binaryImage, filterImage, 25, 25*2, 25/2);

        //imshow("binaryImage", filterImage);
        //waitKey(10);

        //【4】进行霍夫圆变换
        vector<Vec3f> circles;
        HoughCircles(filterImage, circles, HOUGH_GRADIENT, 1, 20, 200, 60, 80, 0);

        //【5】依次在图中绘制出圆
        for( size_t i = 0; i < circles.size(); i++ )
        {
            //参数定义
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            cout << "center:[" << circles[i][0] << ", " << circles[i][1] << "]   ,  "  << "radius:" << circles[i][2] << endl;
            //绘制圆心
            circle(filterImage, center, 3, Scalar(255), -1, 8, 0 );
            //绘制圆轮廓
            circle(filterImage, center, radius, Scalar(255), 3, 8, 0 );
        }

        // //【6】显示效果图  
        imshow("result", filterImage);  
        waitKey(10);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", filterImage).toImageMsg();
        pub.publish(msg); 
        
        ros::spinOnce();
        rate_.sleep();
    }
        

        return 0;  
}