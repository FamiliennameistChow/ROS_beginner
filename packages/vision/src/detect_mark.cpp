#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <drone_flight_modes.hpp>
#include <vision/detResult.h>

#include <iostream>
#include <math.h>
#include <string.h>
#include <chrono>

using namespace cv;
using namespace std;

static void version()
{
    cout << "Using OpenCV version\n" << CV_VERSION << "\n" << endl;
}

// 全局变量
vision::detResult det_result;
Mat img_sub;
int thresh = 50, N = 5;
// const char* wndname = "Square Detection Demo";

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
        }
        // cout << "point: " << "x:" << p-> x << "y:" << p->y << endl;
        //dont detect the border
        if ((p-> x > 3 && p->y > 3) && (abs(p_last-> x - p-> x) > 3) && (i!= 0))
        {
            // polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);
            fillPoly(mask, &p, &n, 1, Scalar(255, 255, 255), 8, 0);
            p_last = p;
        }

    }

//    imshow(wndname, image);
}

void detectLandmark(Mat& image, Mat& mask, Point& center)
{
    det_result.success = false;
    Point img_center(image.cols/2, image.rows/2);
    Mat image_masked, image_gray, image_blur, image_thres;
    bitwise_and(image, image, image_masked, mask);
    // imshow("image_masked", image_masked);
    cvtColor(image_masked, image_gray, CV_BGR2GRAY);
    threshold(image_gray, image_thres, 0, 255, THRESH_BINARY);
    // imshow("image_thres", image_thres);
    GaussianBlur(image_thres, image_blur, Size(3, 3), 2, 2);
    // imshow("image_blur", image_blur);
    vector<Vec3f> circles;
    HoughCircles(image_gray, circles, CV_HOUGH_GRADIENT, 1, image.rows/20, 100, 100, 0, 0);
    cout <<"detected:" <<  circles.size() << endl;
    for(size_t i=0; i<circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        circle(image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        circle(image, center, radius, Scalar(155, 50, 255), 3, 8, 0);
        int error_x = center.x - img_center.x;
        int error_y = img_center.y - center.y;
        cout << "x: "<< error_x << "  y: " << error_y << endl;
        det_result.x_err = error_x;
        det_result.y_err = error_y;
        det_result.success = true;
    }
}


int main(int argc, char** argv)
{
    // static const char* names[] = { "../11.png", 0 };
    version();
    // namedWindow( wndname, 1 );

    ros::init(argc, argv, "landmark_detect");
    ros::NodeHandle nh;
    ros::Rate rate_ = ros::Rate(20.0);

    //发布处理结果
    ros::Publisher detResult_pub = nh.advertise<vision::detResult>("auto_landing/detResult", 100);

    //image_transport 负责图像发布与订阅
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/iris/usb_cam_down/image_raw", 1, imageCallback);
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
    Mat mask = Mat::zeros(image.size(), CV_8UC1);
    Mat img;
    Point center;
    
    while(ros::ok())
    {    
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        img_sub.copyTo(image);
        image.copyTo(img);
        findSquares(image, squares);
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
        cout << "findSquares time cost:    " << time_findSquares.count()   << endl;
        cout << "drawSquares time cost:    " << time_drawSquares.count()    << endl;
        cout << "detectLandmark time cost: " << time_detectLandmark.count() << endl;
        cout << "detect time cost:         " << time_used_total.count()     << endl;

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        pub.publish(msg);

        detResult_pub.publish(det_result);
        //imwrite( "out", image );
        // int c = waitKey();
        // if( (char)c == 27 )
        //     break;
        ros::spinOnce();
        rate_.sleep();
    }

    return 0;
}

