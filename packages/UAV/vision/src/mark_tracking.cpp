/***************************************************************************************************************************
 * mark_tracking.cpp
 *
 * Author: Danny
 *
 * Update Time: 2019.11.25
 *
 * 说明: 基于单目摄像头的　自主目标追踪　和　降落　程序
 *      1. 订阅目标相对位置(来自视觉的ros节点)
 *      2. 动态目标追踪算法及降落策略
***************************************************************************************************************************/
#include <ros/ros.h>
#include <drone_flight_modes.hpp>
#include <vision/detResult.h>

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

float kpx_land, kpy_land, kpz_land; //降落追踪控制算法的 速度 比例项参数
float kix_land, kiy_land, kiz_land; //降落追踪控制算法的 速度 积分项参数
float kdx_land, kdy_land, kdz_land; //降落追踪控制算法的 速度 微分项参数
float kpx_land2, kpy_land2, kpz_land2; //低高度时降落追踪控制算法的 速度 比例项参数
float kix_land2, kiy_land2, kiz_land2; //低高度时降落追踪控制算法的 速度 积分项参数
float kdx_land2, kdy_land2, kdz_land2; //低高度时降落追踪控制算法的 速度 微分项参数
float Max_velx_land, Max_vely_land, Max_velz_land; //降落追踪控制算法的最大速度
float Thres_velx_land, Thres_vely_land, Thres_velz_land;   
float fly_min_z; //允许飞行最低高度[这个高度是指降落板上方的相对高度]
float fly_max_z; //允许飞行最大高度[这个高度是指降落板上方的相对高度]
bool Flag_reach_pad_center = false; //是否到达目标点中心的标志位
bool Flag_z_below_20cm = false; //高度是否达到允许降落的标志位
float Thres_distance_land; //允许降落最大距离阈值
float Thres_count_land; //允许降落计数阈值
float Thres_count_z; //调整高度计数阈值
float land_max_z; //允许降落最大高度阈值
float Thres_vision_lost; //视觉丢失计数阈值
float takeoff_hgt;
float mark_hight;
float error_sum_x = 0, error_sum_y = 0; //降落追踪控制算法的速度积分项
float derror_x = 0, derror_y = 0; //降落追踪控制算法的速度微分项
float x_err = 0, y_err = 0; // 无人机与降落板之间的位置偏差（单位：米）
float hight, hight_prev;
float dt;
bool hgt_sw = true;

int num_count = 0; //允许降落计数
int num_count_lost = 0;  //视觉丢失计数
int num_count_z = 0; //降落过程中偏离理想高度的计数
float distance_pad = 100; //无人机与降落板中心的距离
float vx, vy, vz;
bool Flag_first_result = true;
float x_begin, y_begin, z_begin, distance_pad_begin;
float fx, fy, cx, cy;
chrono::steady_clock::time_point last_result_time, last_loop_time;
vision::detResult det_result;
vision::detResult det_result_prev;

// 获得　地标中心点　的归一化坐标
void pixelToNED(vision::detResult &det_result, Quaterniond &q)
{
    Vector3d body_rotated(cy - det_result.y_err, cx - det_result.x_err, 1);
    // ENU坐标系下的矫正坐标(单位四元数取共轭即为逆)
    Vector3d body = q.conjugate() * body_rotated;
    // 再转换成NED坐标
    det_result.x_err = -body(1) / fx;
    det_result.y_err = body(0) / fy;
    // cout << "x:" << det_result.x_err << ", y:" << det_result.y_err << endl;
}

void detResultCB(const vision::detResult::ConstPtr &msg, Quaterniond &q)
{
    last_result_time = chrono::steady_clock::now();
    det_result = *msg;
    pixelToNED(det_result, q);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_landing");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    AeroDrone myDrone;

    Quaterniond q(myDrone.localPosition().pose.orientation.w, myDrone.localPosition().pose.orientation.x, 
                  myDrone.localPosition().pose.orientation.y, myDrone.localPosition().pose.orientation.z);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //降落追踪控制算法 的比例参数
    nh.param<float>("kpx_land", kpx_land, 0.3);
    nh.param<float>("kpy_land", kpy_land, 0.3);
    nh.param<float>("kpz_land", kpz_land, 0.3);

    //降落追踪控制算法 的比例参数
    nh.param<float>("kix_land", kix_land, 0.3);
    nh.param<float>("kiy_land", kiy_land, 0.3);
    nh.param<float>("kiz_land", kiz_land, 0.3);

    //降落追踪控制算法 的比例参数
    nh.param<float>("kdx_land", kdx_land, 0.3);
    nh.param<float>("kdy_land", kdy_land, 0.3);
    nh.param<float>("kdz_land", kdz_land, 0.3);

    //低高度时降落追踪控制算法 的比例参数
    nh.param<float>("kpx_land2", kpx_land2, 0.3);
    nh.param<float>("kpy_land2", kpy_land2, 0.3);
    nh.param<float>("kpz_land2", kpz_land2, 0.3);

    //低高度时降落追踪控制算法 的比例参数
    nh.param<float>("kix_land2", kix_land2, 0.3);
    nh.param<float>("kiy_land2", kiy_land2, 0.3);
    nh.param<float>("kiz_land2", kiz_land2, 0.3);

    //低高度时降落追踪控制算法 的比例参数
    nh.param<float>("kdx_land2", kdx_land2, 0.3);
    nh.param<float>("kdy_land2", kdy_land2, 0.3);
    nh.param<float>("kdz_land2", kdz_land2, 0.3);

    //降落追踪控制算法的最大速度
    nh.param<float>("Max_velx_land", Max_velx_land, 0.3);
    nh.param<float>("Max_vely_land", Max_vely_land, 0.3);
    nh.param<float>("Max_velz_land", Max_velz_land, 0.3);

    //降落追踪控制算法的速度死区
    nh.param<float>("Thres_velx_land", Thres_velx_land, 0.02);
    nh.param<float>("Thres_vely_land", Thres_vely_land, 0.02);
    nh.param<float>("Thres_velz_land", Thres_velz_land, 0.02);

    //允许降落最大距离阈值
    nh.param<float>("Thres_distance_land", Thres_distance_land, 0.2);

    //允许降落计数阈值
    nh.param<float>("Thres_count_land", Thres_count_land, 20);

    //允许降落计数阈值
    nh.param<float>("Thres_count_z", Thres_count_z, 20);

    //允许降落最大高度阈值
    nh.param<float>("land_max_z", land_max_z, 0.6);

    //允许飞行最低高度[这个高度是指降落板上方的相对高度]
    nh.param<float>("fly_min_z", fly_min_z, 0.3);

    //允许飞行最大高度[这个高度是指降落板上方的相对高度]
    nh.param<float>("fly_max_z", fly_max_z, 8);

    //视觉丢失计数阈值
    nh.param<float>("Thres_vision_lost", Thres_vision_lost, 30);

    //起飞高度
    nh.param<float>("takeoff_hgt", takeoff_hgt, 4);

    //降落板高度
    nh.param<float>("mark_hight", mark_hight, 0.5);

    //相机内参
    nh.param<float>("fx", fx, 400.0);
    nh.param<float>("fy", fy, 277.191356);
    nh.param<float>("cx", cx, 640);
    nh.param<float>("cy", cy, 360);


    
    ros::Subscriber detResult_sub = nh.subscribe<vision::detResult>("auto_landing/detResult", 10, boost::bind(&detResultCB, _1, q));

    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = 0;
    setpoint.pose.position.y = 0;
    setpoint.pose.position.z = takeoff_hgt;

    myDrone.arm();
    sleep(1);

    // set to offboard mode 
    if (!myDrone.setMode("OFFBOARD"))
    {
      ROS_ERROR("Fatal: Set OFFBOARD mode failed!");
    }
    else
    {
      ROS_INFO("Set OFFBOARD mode sent");
    }

    // publish a setpoint 
    myDrone.pubLocalPos(setpoint);
    sleep(5);

    last_loop_time = chrono::steady_clock::now();

    while(ros::ok())
    {
        q.x() = myDrone.localPosition().pose.orientation.x;
        q.y() = myDrone.localPosition().pose.orientation.y;
        q.z() = myDrone.localPosition().pose.orientation.z;
        q.w() = myDrone.localPosition().pose.orientation.w;

        dt = (chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - last_loop_time)).count();
        last_loop_time = chrono::steady_clock::now();

        if(((chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - last_result_time)).count()) >= 1)
        {
            cout << "超过一秒钟未接收到识别结果，无人机悬停在当前位置" << endl;
            myDrone.setVelocityBody(0, 0, 0, 0);
            continue;
        }

        // 将归一化坐标转化为实际坐标，需要无人机的高度信息(相对于降落板的高度)
        hight = myDrone.localPosition().pose.position.z - mark_hight;
        // cout << "相对于降落板的高度为：" << hight << endl;

        x_err = hight * det_result.x_err;
        y_err = hight * det_result.y_err;
        //计算与降落板之间的距离
        distance_pad = sqrt(x_err * x_err + y_err * y_err);

        // Integral action
        error_sum_x += (x_err * dt);
        error_sum_y += (y_err * dt);

        // Derivative action
        derror_x = (x_err - hight_prev *  det_result_prev.x_err) / dt;
        derror_y = (y_err - hight_prev * det_result_prev.y_err) / dt;

        if(det_result.success)
        {
            if(Flag_first_result)
            {
                x_begin = x_err;
                y_begin = y_err;
                distance_pad_begin = distance_pad;
                z_begin = myDrone.localPosition().pose.position.z - mark_hight;
                Flag_first_result = false;
            }

            num_count_lost = 0;

            vz = -0.3;
            if(hight >= 1.3)
            {
                vx = kpx_land * x_err + kix_land * error_sum_x + kdx_land * derror_x;
                vy = kpy_land * y_err + kiy_land * error_sum_y + kdy_land * derror_y;
                hgt_sw = true;
            }
            else
            {
                if(hgt_sw)
                {
                    error_sum_x = 0;
                    error_sum_y = 0;
                }
                vx = kpx_land2 * x_err + kix_land2 * error_sum_x + kdx_land2 * derror_x;
                vy = kpy_land2 * y_err + kiy_land2 * error_sum_y + kdy_land2 * derror_y; 
                hgt_sw = false;             
            }
            vx = (abs(vx) > Max_velx_land ?  (vx > 0 ? Max_velx_land : -Max_velx_land) : (vx));
            vy = (abs(vy) > Max_vely_land ?  (vy > 0 ? Max_vely_land : -Max_vely_land) : (vy));
            // 如果相对距离小于阈值，计数增加
            if(distance_pad < Thres_distance_land)
            {
                num_count++;
                cout << "到达地标位置，无人机下降" << endl;
                // vx *= 1.2;
                // vy *= 1.2;
                if(hight < 0.5)
                    vz = -0.8;
                else
                    vz = -0.5;
            }
            else
                num_count = 0;

            myDrone.setVelocityBody(vx, vy, vz, 0);
        }
        else
        {
            num_count_lost++;
            error_sum_x = 0;
            error_sum_y = 0;

            //如果丢失计数超过阈值，则在不超过最大高度的前提下，无人机上升以获得更大的视野
            if(num_count_lost > Thres_vision_lost)
            {
                num_count = 0;
                if(hight <= fly_max_z)
                    vz = 1;
                else
                    vz = 0;
                myDrone.setVelocityBody(0, 0, vz, 0);
            }
        }

        //降落的2个条件：
        //1：水平距离小于阈值，即无人机抵达降落板中心位置附近
        //2：相对高度小于阈值，

        // 如果相对距离小于阈值，计数增加
        // if(distance_pad < Thres_distance_land && det_result.success)
        // {
        //     num_count++;
        //     cout << "到达地标位置，无人机下降" << endl;
        //     // cout<< "Distance_pad: " << distance_pad <<endl;
        //     // cout<< "Distance_land_count: " << num_count <<endl;
        // }
        // else
        // {
        //     num_count = 0;
        //     // cout<< "Distance_pad: " << distance_pad <<endl;
        //     // cout<< "Distance_land_count: " << num_count <<endl;
        // }

        //如果计数增加超过阈值，则满足降落的第一个条件（水平距离）
        if(distance_pad <= Thres_distance_land && num_count >= Thres_count_land)
        {
            cout<< "Flag_reach_pad_center: " << "true" <<endl;
            Flag_reach_pad_center = true;
        }
        else
        {
            // cout<< "Flag_reach_pad_center: " << "flase" <<endl;
            Flag_reach_pad_center = false;
        }

        //如果 相对高度 小于 阈值，则满足降落的第二个条件（高度条件）
        if(hight <=  land_max_z)
        {
            cout<< "Flag_z_below_: " << "true" <<endl;
            Flag_z_below_20cm = true;
        }
        else
        {
            // cout<< "Flag_z_below_20cm: " << "flase" <<endl;
            Flag_z_below_20cm = false;
        }

        //如果降落的两个条件都满足，则降落并上锁
        if(Flag_z_below_20cm)
        {
            // land and disarm!
            // myDrone.land();
            // sleep(3);

            if (!myDrone.disarm())
            {
                ROS_ERROR("Fatal: disarm failed!");
            }
            else
            {
                ROS_INFO("disarm sent");
                sleep(3);
                break;
            }
        }

        det_result_prev = det_result;
        hight_prev = hight;

        ros::spinOnce();
        loop_rate.sleep();
    }
}