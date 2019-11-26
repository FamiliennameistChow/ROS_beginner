/***************************************************************************************************************************
 * detect_mark.cpp
 *
 * Author: Chow
 *
 * Update Time: 2019.11.19
 *
 * 说明: 地标识别程序
 *      1. 【订阅】下置摄像头图像数据
 *      2. 【发布】识别结果 <vision::detResult> 
 *      2. 【发布】识别后的图像 "auto_landing/image/detect_result" 
***************************************************************************************************************************/
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/ml/ml.hpp>
#include <opencv2/ml.hpp>

#include <drone_flight_modes.hpp>
#include <vision/detResult.h>

#include <iostream>
#include <math.h>
#include <string.h>
#include <chrono>
#include <unistd.h>

using namespace cv;
using namespace std;
using namespace cv::ml;


// 全局变量
vision::detResult det_result;
Mat img_sub;
int thresh = 50, N = 5;
int Mark_type;

char szPath[128];
void getPath()
{
    memset( szPath, 0x00, sizeof(szPath));
    // getcwd(szBuf, sizeof(szBuf)-1);
    int ret =  readlink("/proc/self/exe", szPath, sizeof(szPath)-1 );
    printf("path:%s\n", szPath);
    int len = strlen(szPath) - 1;
    int count_=0;
    for (int j=len; j>0; j--)
    {
        if (szPath[j] == '/')
        {
            count_++;
        }
        if (count_ == 4)
        {
            szPath[j+1] = 0;
            cout << "dir:" << szPath << endl;
            break;
        }
    }
}


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


// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}


static void approxRectangle (const Mat& image, const Mat& img_process, vector<vector<Point> >& squares)
{
// *************针对矩形标靶，拟合矩形框****************
// input image: raw image
// input img_process: 单通道图像
// input squares: 矩形四个角点点集
//
//***************************************************
    int img_area = image.cols * image.rows;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<Point> approx;
    findContours(img_process, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    // cout << "contours num: " << contours.size() << endl;
    for (size_t i = 0; i < contours.size(); i++)
    {
        int area = contourArea(contours[i]);
        if (area < 500) continue;  // contours whos area less than 1000 pixel will be passed!!
        // Scalar color(rand() & 255, rand() & 255, rand()&255);
        // cout << "area" <<  area << endl;
        // drawContours(image, contours, i, color, FILLED, 8, hierarchy);
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
        if (approx.size() == 4 &&
            isContourConvex(Mat(approx)))
        {
            double maxCosine = 0;

            for( int j = 2; j < 5; j++ )
            {
                // find the maximum cosine of the angle between joint edges
                double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                maxCosine = MAX(maxCosine, cosine);
            }

            // if cosines of all angles are small
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
            if( maxCosine < 0.3 )
                squares.push_back(approx);
        }
    }
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static void findSquares( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

    // Mat pyr, timg, gray0(image.size(), CV_8U), gray;

    // down-scale and upscale the image to filter out the noise
    //pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    //pyrUp(pyr, timg, image.size());


    // blur will enhance edge detection
    Mat timg(image);
    medianBlur(image, timg, 9);
    Mat gray0(timg.size(), CV_8U), gray;
    int img_area = image.cols * image.rows;
    cvtColor(timg, timg, COLOR_BGR2HSV);

    vector<vector<Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 5, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

            vector<Point> approx;

            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 &&
                    (fabs(contourArea(Mat(approx))) < (img_area/2))  &&
                    isContourConvex(Mat(approx)) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
        }
    }
}

static void findSquaresHSV( const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();
    int img_area = image.cols * image.rows;
    Mat img_hsv;
    cvtColor(image, img_hsv, COLOR_BGR2HSV);
    vector<Mat> hsv;
    Mat canny, gray;
    split(img_hsv, hsv);
    for (int c=0; c < 3; c++)
    {
        string str_c = to_string(c);
        // imshow("hsv_" + str_c, hsv[c]);
        if (c == 2)  // process v Channel
        {
            for (int l=0; l < N; l++)
            {
                string str_l = to_string(l);
                if(l==0)
                {
                    Canny(hsv[c], canny, 5, thresh, 5);
                    dilate(canny, gray, Mat(), Point(-1,-1));
                    threshold(gray, gray, 0, 255, THRESH_BINARY_INV);
                    // imshow("img"+ str_c+"_"+ str_l, gray);
                }
                else
                {
                    gray = hsv[c] >= (l+1)*255/N;
                    // imshow("img"+ str_c+"_"+ str_l, gray);
                }
                approxRectangle(image, gray, squares);
            }
        }
        else  //process h, s channel
        {
            Canny(hsv[c], canny, 5, thresh, 5);
            dilate(canny, gray, Mat(), Point(-1,-1));
            threshold(gray, gray, 0, 255, THRESH_BINARY_INV);
            approxRectangle(image, gray, squares);
            // imshow("img_"+ str_c , gray);
        }

    }

//    imshow("image_contours", image);
//    for(auto it=squares.begin();it!=squares.end();it++)
//    cout<< "point:" <<*it<<endl;

}


// the function draws all the squares in the image
static void drawSquares( Mat& image, const vector<vector<Point> >& squares, Mat& mask)
{
    const Point* p_last;
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        if (i==0)
        {
            fillPoly(mask, &p, &n, 1, Scalar(255, 255, 255), 8, 0);
            p_last = p;
            // cout<<"+++"<< endl;
        }
        // cout << "point: " << "x:" << p-> x << "y:" << p->y << endl;
        //dont detect the border
        if ((p-> x > 3 && p->y > 3) && (abs(p_last-> x - p-> x) > 3) && (i!= 0))
        {
            // polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);
            fillPoly(mask, &p, &n, 1, Scalar(255, 255, 255), 8, 0);
            p_last = p;
            // cout << "----"<< endl;
        }
        // cout << "====end====" << endl;

    }

    // imshow("image_Squares", image);
}

// 20191124 modify
void detectLandmark(Mat& image, Mat& mask, Point& center)
{
    det_result.success = false;
//    imshow("mask", mask);
    cout << "the detect Mark_type is :" << Mark_type << endl;
    Mat image_masked, image_gray, image_thres;
    bitwise_and(image, image, image_masked, mask);
    // imshow("image_masked", image_masked);
    cvtColor(image_masked, image_gray, CV_BGR2GRAY);
//    GaussianBlur(image_gray, image_blur, Size(3, 3), 2, 2);
//    imshow("image_blur", image_blur);
    // detect Num mark Type
    if (Mark_type == 0)
    {
        vector<Vec3f> circles;
        HoughCircles(image_gray, circles, CV_HOUGH_GRADIENT, 1, image.rows/20, 100, 100, 0, 0);
        cout <<"detected:" <<  circles.size() << endl;
        for(size_t i=0; i<circles.size(); i++)
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            circle(image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            circle(image, center, radius, Scalar(155, 50, 255), 3, 8, 0);
            det_result.x_err = center.x;
            det_result.y_err = center.y;
            det_result.success = true;
        }
    }
    // detect Num mark Type
    if (Mark_type == 3)
    {
        Ptr<KNearest> knn(ml::KNearest::create());
        try
        {
            // cout<<"there is " << strcat(szPath, "src/vision/config/models.yml") << endl;
            // auto dir = strcat(szPath, "src/vision/config/models.yml");
            knn = Algorithm::load<KNearest>("/home/danny/catkin_ws/src/vision/config/models.yml");
        }  
        catch(Exception)
        {
            cout << "=======" << endl;
            cout << "MODEL LOAD ERROR: there is no models.yml file in config folder" << endl;
        }
        Mat image_erode;
        Rect rect;
        bool detectNum = false;
        // imshow("image_gray", image_gray);
        erode(image_gray, image_erode, Mat(), Point(-1,-1));
        // imshow("erode", image_erode);
        vector<Vec4i> hierarchy;
        vector<vector<Point> > contours;
        findContours(image_erode, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE);
        cout << "detected: " << contours.size() << endl;

//        for(auto it=hierarchy.begin();it!=hierarchy.end();it++)
//        cout<< "hierarchy   :" <<*it<<endl;
//        drawContours(image, contours, 0, Scalar(255, 255, 0), 1, 8, hierarchy);
//        cout << "hierarchy" << hierarchy[0] <<endl;

        for (size_t i=0; i<contours.size(); i++)
        {
            int area = contourArea(contours[i]);
            // cout << "area " << area << endl;
            if (area < 200) continue;

            if (hierarchy[i][3] != -1) //有父轮廓
            {
                rect = boundingRect(contours[i]);
                detectNum = true;
            }

//            Scalar color(rand() & 255, rand() & 255, rand()&255);
//            drawContours(image, contours, i, color, 1, 8, hierarchy);
//            cout << "hierarchy" << hierarchy[i] <<endl;
        }
        if (detectNum)
        {
            // idetify num
            Mat ROI = image_gray(rect);
            threshold(ROI, ROI, 0, 255, THRESH_BINARY_INV);
            // imshow("ROI", ROI);
//            cout << "W: " << ROI.cols << " H: " << ROI.rows << endl;
            Mat temp, temp2;
            resize(ROI, temp, Size(56, 56), 0, 0, INTER_LINEAR);
            temp.convertTo(temp2, CV_32FC1);
            Mat response;
            int p = knn->findNearest(temp2.reshape(1, 1), 1, response);
            cout << "detected num:      " << p << endl;
            det_result.x_err = (rect.tl().x + rect.br().x) / 2;
            det_result.y_err = (rect.tl().y + rect.br().y) / 2;
            det_result.success = true;
            det_result.num = p;
            // draw result
            rectangle(image, rect.tl(), rect.br(), Scalar(155, 50, 255), 2);
            circle(image, Point(det_result.x_err, det_result.y_err), 3, Scalar(0, 255, 0), -1, 8, 0);
        }
    }
    cout << "x: "<< det_result.x_err << "  y: " << det_result.y_err << endl;
    // imshow("image", image);
}

int main(int argc, char** argv)
{
    cout << "Using OpenCV version\n" << CV_VERSION << "\n" << endl;
    // getPath();
    ros::init(argc, argv, "landmark_detect");
    ros::NodeHandle nh;
    ros::Rate rate_ = ros::Rate(20.0);

    nh.param<int>("Mark_type", Mark_type, 3);

    //【发布】处理结果
    ros::Publisher detResult_pub = nh.advertise<vision::detResult>("auto_landing/detResult", 100);

    //image_transport 负责图像发布与订阅
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/iris/usb_cam2/image_raw", 1, imageCallback);
    image_transport::Publisher pub = it.advertise("auto_landing/image/detect_result", 1);

    while(img_sub.empty())
    {
        cout<<"no image data!!"<<endl;
        ros::spinOnce();
        rate_.sleep();
        continue;
    }

    Mat image;
    vector<vector<Point> > squares;
    Mat img;
    Point center;
    
    while(ros::ok())
    {    
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        img_sub.copyTo(image);
        image.copyTo(img);
        Mat mask = Mat::zeros(image.size(), CV_8UC1);
        // findSquares(image, squares);
        findSquaresHSV(image, squares);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        drawSquares(image, squares, mask);
        chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
        detectLandmark(img, mask, center);
        // imshow("detected", img);
        chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
        chrono::duration<double> time_findSquares    = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
        chrono::duration<double> time_drawSquares    = chrono::duration_cast<chrono::duration<double>>( t3-t2 );
        chrono::duration<double> time_detectLandmark = chrono::duration_cast<chrono::duration<double>>( t4-t3 );
        chrono::duration<double> time_used_total     = chrono::duration_cast<chrono::duration<double>>( t4-t1 );
        // cout << "findSquares time cost:    " << time_findSquares.count()   << endl;
        // cout << "drawSquares time cost:    " << time_drawSquares.count()    << endl;
        // cout << "detectLandmark time cost: " << time_detectLandmark.count() << endl;
        cout << "detect time cost:         " << time_used_total.count()     << endl;

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        pub.publish(msg);

        detResult_pub.publish(det_result);
        //imwrite( "out", image );
        int c = waitKey(1);
        if( (char)c == 27 )
            break;
        ros::spinOnce();
        rate_.sleep();
    }

    return 0;
}

