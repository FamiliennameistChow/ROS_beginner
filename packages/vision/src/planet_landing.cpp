/***************************************************************************************************************************
 * planet_landing.cpp
 *
 * Author: Danny
 *
 * Update Time: 2019.12.10
 *
 * 说明: 行星表面降落比赛的着陆策略代码
 *      1. 订阅四个标靶的相对位置(来自视觉的ros节点)
 *      2. 根据识别得到的着陆区位置进行着陆
***************************************************************************************************************************/
#include <ros/ros.h>
#include <drone_flight_modes.hpp>
#include <vision/redResult.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <algorithm>
#include <string.h>

#include <signal.h>
#include <cmath>
#include <iostream>
#include <time.h>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/bind.hpp>

using namespace std;
using namespace Eigen;

// 整个流程所用到的无人机的状态
namespace Status_n {
enum Status_t {
    WAITING,
    FINGDING,
    MOVING,
    DETECTING,
    LANDING,
    LANDED,
};
}
typedef Status_n::Status_t Status;

struct Geofence
{
    float x_max;
    float x_min;
    float y_max;
    float y_min;
};

bool flag_get_red = false, flag_get_landing = false;
std_msgs::Bool flag_target_right;
float mark_hight, first_hight, second_hight, third_hight;
geometry_msgs::Point mark_center, target_pos;
Status current_status = Status_n::WAITING;
string start_signal;
float fx, fy, cx, cy; // 相机内参
int detect_times = 0;
vision::redResult det_start;
vision::redResult marks_pixel, marks_ENU, landing_target_pixel, landing_target_ENU;
geometry_msgs::PoseStamped setpoint;

void pixelToENU(vision::redResult &coordinate_pixels, vision::redResult &coordinate_ENUs, geometry_msgs::Pose localPosition)
{
    Quaterniond q(localPosition.orientation.w, localPosition.orientation.x, 
                  localPosition.orientation.y, localPosition.orientation.z);
    float hight = localPosition.position.z;

    for(geometry_msgs::Point coordinate_pixel : coordinate_pixels.mark_ori)
    {
        Vector3d body_rotated(cy - coordinate_pixel.y, cx - coordinate_pixel.x, 1);
        // ENU坐标系下的矫正坐标(单位四元数取共轭即为逆)
        Vector3d body = q.conjugate() * body_rotated;

        geometry_msgs::Point coordinate_ENU;
        coordinate_ENU.x = localPosition.position.x + hight * body(0) / fy;
        coordinate_ENU.y = localPosition.position.y + hight * body(1) / fx;
        coordinate_ENUs.mark_ori.push_back(coordinate_ENU);
    }

}

void redResultCB(const vision::redResult::ConstPtr &msg)
{
    marks_pixel = *msg;
    flag_get_red  = true;
}

void landingResultCB(const vision::redResult::ConstPtr &msg)
{
    landing_target_pixel = *msg;
    flag_get_landing = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planet_landing");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    AeroDrone myDrone;
    struct Geofence geofence;

    nh.param<float>("mark_hight", mark_hight, 0.2);
    nh.param<float>("first_hight", first_hight, 6);
    nh.param<float>("second_hight", second_hight, 4);
    nh.param<float>("third_hight", third_hight, 1);

    //相机内参
    nh.param<float>("fx", fx, 536);
    nh.param<float>("fy", fy, 536);
    nh.param<float>("cx", cx, 640);
    nh.param<float>("cy", cy, 360);

    ros::Subscriber red_result_sub = nh.subscribe<vision::redResult>("planet_landing/red_result", 1000, redResultCB);
    ros::Subscriber landing_result_sub = nh.subscribe<vision::redResult>("planet_landing/det_result", 1000, landingResultCB);
    ros::Publisher det_signal_pub = nh.advertise<vision::redResult>("planet_landing/det_start", 1000);
    ros::Publisher targrt_right_pub = nh.advertise<std_msgs::Bool>("planet_landing/target_right", 1000);
    

    while(ros::ok())
    {
        ros::spinOnce();
        
        switch(current_status)
        {
            // 等待开始信号
            case Status_n::WAITING:
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>waiting<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                cout << "请输入 'go' 开始降落流程" << endl;
                cin >> start_signal;

                if(start_signal == "go")
                {
                    current_status = Status_n::FINGDING;
                    break;
                }
                else
                {
                    cout << "请输入正确的单词" << endl;
                    current_status = Status_n::WAITING;
                    break;
                }    

            // 寻找四个红色标靶及中心位置
            case Status_n::FINGDING:
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>finding<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                if (!myDrone.currentState().armed)
                {
                    if (!myDrone.arm())
                        ROS_INFO("Vehicle armed failed!");
                    else
                        ROS_INFO("Vehicle armed");
                }
                if(myDrone.currentState().mode != "OFFBOARD")
                {
                    if (!myDrone.setMode("OFFBOARD"))
                        ROS_ERROR("Fatal: Set OFFBOARD mode failed!");
                    else
                        ROS_INFO("Set OFFBOARD mode successful!");
                }
                myDrone.moveENUto(0, 0, first_hight);
                myDrone.rotateAngleto(0);
                cout << "等待无人机到达高度：" << first_hight << "米..." << endl;
                sleep(6);
                cout << "无人机到达高度：" << first_hight << "米" << endl;
                
                while(!flag_get_red)
                {
                    cout << "还未识别到标靶，向前飞行..." << endl;
                    setpoint.pose.position.x = myDrone.localPosition().pose.position.x + 0.2;
                    setpoint.pose.position.y = 0;
                    setpoint.pose.position.z = first_hight;
                    myDrone.pubLocalPos(setpoint);
                    // myDrone.setVelocityBody(0, 0.3, 0, 0);
                    ros::spinOnce();
                    loop_rate.sleep();
                }

                if(flag_get_red)
                {
                    while(marks_pixel.red_mark_num != 4)
                    {
                        if(myDrone.localPosition().pose.position.x < 8)
                        {
                            cout << "未同时识别到四个标靶，向前飞行..." << endl;
                            // myDrone.setVelocityBody(0, 0.3, 0, 0);
                            setpoint.pose.position.x = myDrone.localPosition().pose.position.x + 0.2;
                            setpoint.pose.position.y = 0;
                            setpoint.pose.position.z = first_hight;
                            myDrone.pubLocalPos(setpoint);
                            ros::spinOnce();
                            loop_rate.sleep();
                        }
                        else break;
                    }
                    // myDrone.setVelocityBody(0, 0, 0, 0);
                    myDrone.rotateAngleto(0);
                    sleep(2);

                    while(marks_pixel.red_mark_num != 4)
                    {
                        if(myDrone.localPosition().pose.position.x > 0)
                        {
                            cout << "超过了合理的位置，往回飞行..." << endl;
                            // myDrone.setVelocityBody(0, -0.15, 0, 0);
                            setpoint.pose.position.x = myDrone.localPosition().pose.position.x - 0.15;
                            setpoint.pose.position.y = 0;
                            setpoint.pose.position.z = first_hight + 0.5;
                            myDrone.pubLocalPos(setpoint);
                            ros::spinOnce();
                            loop_rate.sleep();
                            sleep(2);
                        }
                        else
                        {
                            cout << "无法搜寻到四个标靶，准备降落..." << endl;
                            // myDrone.setVelocityBody(0, 0, 0, 0);
                            sleep(2);
                            current_status = Status_n::LANDING;
                            goto next;
                        }
                    }
                }
                
                flag_get_red = false;
                cout << "识别到了四个标靶，坐标转换中..." << endl;
                pixelToENU(marks_pixel, marks_ENU, myDrone.localPosition().pose);
                geofence.x_max = max(max(max(marks_ENU.mark_ori[0].x, marks_ENU.mark_ori[1].x), marks_ENU.mark_ori[2].x), marks_ENU.mark_ori[3].x);
                geofence.x_min = min(min(min(marks_ENU.mark_ori[0].x, marks_ENU.mark_ori[1].x), marks_ENU.mark_ori[2].x), marks_ENU.mark_ori[3].x);
                geofence.y_max = max(max(max(marks_ENU.mark_ori[0].y, marks_ENU.mark_ori[1].y), marks_ENU.mark_ori[2].y), marks_ENU.mark_ori[3].y);
                geofence.y_min = min(min(min(marks_ENU.mark_ori[0].y, marks_ENU.mark_ori[1].y), marks_ENU.mark_ori[2].y), marks_ENU.mark_ori[3].y);

                cout<< "x_max: " << geofence.x_max << " , x_min: " << geofence.x_min << " , y_max: " << geofence.y_max << ", y_min: " << geofence.y_min << endl;
                mark_center = marks_ENU.mark_ori[marks_ENU.mark_ori.size() - 1];
                target_pos = mark_center;
                target_pos.z = first_hight;
                cout << "坐标转换完成，无人机飞往沙盘中心点..." << endl;
                marks_ENU.mark_ori.clear();
                current_status = Status_n::MOVING;          
                
                next:
                    break;
            
            case Status_n::MOVING:
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>moving<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                myDrone.moveENUto(target_pos.x, target_pos.y, target_pos.z);
                sleep(10);
                cout << "无人机已到达目标点" << endl;
                if(target_pos.z < 1.5)
                {
                    current_status = Status_n::LANDING;
                    break;
                }
                det_start.inCenter = true;
                det_signal_pub.publish(det_start);
                current_status = Status_n::DETECTING;
                break;

             case Status_n::DETECTING:
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>detecting<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                if(flag_get_landing)
                {
                    detect_times++;
                    if(detect_times == 1)
                    {
                        cout << "识别到了初步的着陆区，坐标转换中..." << endl;
                        pixelToENU(landing_target_pixel, landing_target_ENU, myDrone.localPosition().pose);
                        // mark_center.x = landing_target_ENU.mark_ori[0].x > myDrone.localPosition().pose.position.x ? ((geofence.x_max + myDrone.localPosition().pose.position.x)/2) : ((geofence.x_min + myDrone.localPosition().pose.position.x)/2);
                        // mark_center.y = landing_target_ENU.mark_ori[0].y > myDrone.localPosition().pose.position.y ? ((geofence.y_max + myDrone.localPosition().pose.position.y)/2) : ((geofence.y_min + myDrone.localPosition().pose.position.y)/2);
                        // target_pos = mark_center;
                        target_pos = landing_target_ENU.mark_ori[landing_target_ENU.mark_ori.size() - 1];
                        target_pos.z = second_hight;
                        cout << "坐标转换完成，无人机飞往备降区中心点..." << endl;
                        landing_target_ENU.mark_ori.clear();
                        flag_get_landing = false;
                    }
                    if(detect_times > 1)
                    {
                        cout << "识别到了最终的着陆点，坐标转换中..." << endl;
                        pixelToENU(landing_target_pixel, landing_target_ENU, myDrone.localPosition().pose);
                        if((landing_target_ENU.mark_ori[0].x > geofence.x_min) && (landing_target_ENU.mark_ori[0].x < geofence.x_max) && (landing_target_ENU.mark_ori[0].y > geofence.y_min) && (landing_target_ENU.mark_ori[0].y < geofence.y_max))
                        {
                            cout << "坐标转换完成，着陆点在沙盘内，无人机飞往着陆点上方..." << endl;
                            flag_target_right.data = true;
                            targrt_right_pub.publish(flag_target_right);
                            target_pos = landing_target_ENU.mark_ori[landing_target_ENU.mark_ori.size() - 1];
                            target_pos.z = third_hight;
                            landing_target_ENU.mark_ori.clear();
                            flag_get_landing = false;
                        }
                        else
                        {
                            cout << "坐标转换完成，着陆点在沙盘外，重新识别..." << endl;
                            flag_target_right.data = false;
                            targrt_right_pub.publish(flag_target_right);
                            target_pos = landing_target_ENU.mark_ori[landing_target_ENU.mark_ori.size() - 1];
                            target_pos.z = third_hight;
                            landing_target_ENU.mark_ori.clear();
                            flag_get_landing = false;
                            break;
                        }
                    }

                    current_status = Status_n::MOVING;
                    break;
                }
                else
                {
                    cout << "还未识别到着陆区位置，等待..." << endl;
                    break;
                }

            case Status_n::LANDING:
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>landing<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                cout << "无人机已经到达着陆点上方，准备着陆..." << endl;
                myDrone.land();
                sleep(10);
                myDrone.disarm();
                current_status = Status_n::LANDED;
                break;

            case Status_n::LANDED:
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>landed<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                if(myDrone.currentState().armed)
                {
                    cout << "无人机仍未上锁" << endl;
                    current_status = Status_n::LANDING;
                    break;
                }
                else
                {
                    cout << "无人机已经上锁，结束" << endl;
                    break;
                }
        }
        loop_rate.sleep();
    }
        
    return 0;
}
