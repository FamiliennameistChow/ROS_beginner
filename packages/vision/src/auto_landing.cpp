/***************************************************************************************************************************
 * auto_landing.cpp
 *
 * Author: Danny
 *
 * Update Time: 2019.11.20
 *
 * 说明: 基于单目摄像头的自主降落程序
 *      1. 订阅目标相对位置(来自视觉的ros节点)
 *      2. 追踪算法及降落策略
***************************************************************************************************************************/
#include <ros/ros.h>
#include <drone_flight_modes.hpp>
#include <vision/detResult.h>

#include <signal.h>
#include <cmath>
#include <iostream>
#include <time.h>
#include <chrono>
// #include <Eigen/Core>
// #include <Eigen/Geometry>

using namespace std;
// using namespace Eigen;

float kpx_land, kpy_land, kpz_land; ////降落追踪控制算法的 速度 比例参数
float Max_velx_land, Max_vely_land, Max_velz_land; //降落追踪控制算法的最大速度
float Thres_velx_land, Thres_vely_land, Thres_velz_land;   
float fly_min_z; //允许飞行最低高度[这个高度是指降落板上方的相对高度]
float fly_max_z; //允许飞行最大高度[这个高度是指降落板上方的相对高度]
bool Flag_reach_pad_center = false; //是否到达目标点中心的标志位
bool Flag_z_below_20cm = false; //高度是否达到允许降落的标志位
float Thres_distance_land; //允许降落最大距离阈值
float Thres_count_land; //允许降落计数阈值
float land_max_z; //允许降落最大高度阈值
float Thres_vision_lost; //视觉丢失计数阈值
float takeoff_hgt;

int num_count = 0; //允许降落计数
int num_count_lost = 0;  //视觉丢失计数
int num_count_z = 0; //降落过程中偏离理想高度的计数
float distance_pad = 100; //无人机与降落板中心的距离
float vx, vy, vz;
chrono::steady_clock::time_point last_result_time;
bool Flag_first_result = true;
float x_begin, y_begin, z_begin, distance_pad_begin;
float fx, fy, cx, cy;
vision::detResult det_result;

// 获得　地标中心点　的归一化坐标
void pixelToCamera(float u, float v, vision::detResult &det_result)
{
    // Vector3d pixel(u, v, 1);
    // Matrix3d K;
    // K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    // Vector3d camera = K.inverse() * pixel;
    det_result.x_err = (u - cx) / fx;
    det_result.y_err = -(v - cy) / fy;
    // cout << "x:" << det_result.x_err << ", y:" << det_result.y_err << endl;
}

void detResultCB(const vision::detResult::ConstPtr &msg)
{
    last_result_time = chrono::steady_clock::now();
    det_result.success = msg -> success;
    pixelToCamera(msg -> x_err, msg -> y_err, det_result);
    // det_result = *msg;
    //计算与降落板之间的距离
    distance_pad = sqrt(det_result.x_err * det_result.x_err + det_result.y_err * det_result.y_err);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_landing");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    AeroDrone myDrone;

    float hight;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //降落追踪控制算法 的比例参数
    nh.param<float>("kpx_land", kpx_land, 0.3);
    nh.param<float>("kpy_land", kpy_land, 0.3);
    nh.param<float>("kpz_land", kpz_land, 0.3);

    //降落追踪控制算法的最大速度
    nh.param<float>("Max_velx_land", Max_velx_land, 0.3);
    nh.param<float>("Max_vely_land", Max_vely_land, 0.3);
    nh.param<float>("Max_velz_land", Max_velz_land, 0.3);

    //降落追踪控制算法的速度死区
    nh.param<float>("Thres_velx_land", Thres_velx_land, 0.02);
    nh.param<float>("Thres_vely_land", Thres_vely_land, 0.02);
    nh.param<float>("Thres_velz_land", Thres_velz_land, 0.02);

    //允许降落最大距离阈值
    nh.param<float>("Thres_distance_land", Thres_distance_land, 0.4);

    //允许降落计数阈值
    nh.param<float>("Thres_count_land", Thres_count_land, 20);

    //允许降落最大高度阈值
    nh.param<float>("land_max_z", land_max_z, 1.1);

    //允许飞行最低高度[这个高度是指降落板上方的相对高度]
    nh.param<float>("fly_min_z", fly_min_z, 1);

    //允许飞行最低高度[这个高度是指降落板上方的相对高度]
    nh.param<float>("fly_max_z", fly_max_z, 8);

    //视觉丢失计数阈值
    nh.param<float>("Thres_vision_lost", Thres_vision_lost, 30);

    //降落板高度
    nh.param<float>("takeoff_hgt", takeoff_hgt, 4);

    //相机内参
    nh.param<float>("fx", fx, 400.0);
    nh.param<float>("fy", fy, 277.191356);
    nh.param<float>("cx", cx, 640);
    nh.param<float>("cy", cy, 360);


    
    ros::Subscriber detResult_sub = nh.subscribe("auto_landing/detResult", 100, detResultCB);

    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = 0;
    setpoint.pose.position.y = 0;
    setpoint.pose.position.z = takeoff_hgt;

    myDrone.arm();
    sleep(3);

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
    sleep(7);

    while(ros::ok())
    {
        // 将归一化坐标转化为实际坐标
        hight = myDrone.localPosition().pose.position.z;
        cout << "当前高度为：" << hight << endl;

        // (chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - last_result_time)).count();
        if((chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - last_result_time)).count() >= 1)
        {
            cout << "超过一秒钟未接收到识别结果，无人机悬停在当前位置" << endl;
            myDrone.setVelocityBody(0, 0, 0, 0);
            continue;
        }
        if(det_result.success)
        {
            if(Flag_first_result)
            {
                x_begin = hight * det_result.x_err;
                y_begin = hight * det_result.y_err;
                distance_pad_begin = hight * distance_pad;
                // x_begin = det_result.x_err;
                // y_begin = det_result.y_err;
                // distance_pad_begin = distance_pad;
                z_begin = myDrone.localPosition().pose.position.z;
                Flag_first_result = false;
            }

            num_count_lost = 0;

            // 高度直接下降降落的方法
            // vz = abs(kpz_land * (myDrone.localPosition().pose.position.z - fly_min_z)) > Max_velz_land ?  -Max_velz_land : -(kpz_land * (myDrone.localPosition().pose.position.z  - fly_min_z));
            
            // 高度自适应降落的方法
            // 如果　距离地标的距离　大于　允许降落最大距离阈值
            if(hight * distance_pad > Thres_distance_land)
            {
                num_count = 0;

                vx = (abs(kpx_land * hight *  det_result.x_err) > Max_velx_land ?  (det_result.x_err > 0 ? Max_velx_land : -Max_velx_land) : (kpx_land  *hight *  det_result.x_err));
                vy = (abs(kpy_land * hight *  det_result.y_err) > Max_vely_land ?  (det_result.y_err > 0 ? Max_vely_land : -Max_vely_land) : (kpy_land  *hight *  det_result.y_err));

                // 如果　当前高度与第一次识别到地标时高度的比例　和　当前距离与第一次识别到地标时距离的比例　相差较大，认为高度需要调整
                if(abs(hight * distance_pad / distance_pad_begin - (myDrone.localPosition().pose.position.z - fly_min_z) / (z_begin - fly_min_z)) > 0.1)
                {
                    //　对高度是否需要调整计数
                    num_count_z++;

                    // 如果满足调整高度的计数阈值
                    if(num_count_z > 30)
                    {
                        //　如果　距离的比例　大于　高度的比例，则认为高度太低，需要让无人机上升
                        if(hight * distance_pad / distance_pad_begin > (myDrone.localPosition().pose.position.z - fly_min_z) / z_begin){
                            // cout << "无人机上升" << endl;
                            vz = (abs(kpz_land * (myDrone.localPosition().pose.position.z - fly_min_z)) > Max_velz_land ?  Max_velz_land : (kpz_land * (myDrone.localPosition().pose.position.z  - fly_min_z)));
                        }
                        //　如果　距离的比例　小于　高度的比例，则认为高度太低，需要让无人机下降
                        else{
                            // cout << "无人机下降" << endl;
                            vz = (abs(kpz_land * (myDrone.localPosition().pose.position.z - fly_min_z)) > Max_velz_land ?  -Max_velz_land : -(kpz_land * (myDrone.localPosition().pose.position.z  - fly_min_z)));
                        }
                    }
                    // 如果未达到调整高度的计数阈值
                    else{  
                        // cout << "无人机保持高度" << endl;         
                        vz = 0;                
                    }
                }
                // 如果高度合适，则不需要调整高度
                else{
                    // cout << "无人机保持高度" << endl;
                    num_count_z = 0;
                    vz = 0;
                }
            }
            // 如果　距离地标的距离　小于等于　允许降落最大距离阈值，则认为到达地标，直接下降。
            // 且同时满足（distance_pad < Thres_distance_land && det_result.success）,允许降落计数累加
            else{
                num_count++;
                cout << "到达地标位置，无人机下降" << endl;
                vx = 0;
                vy = 0;
                vz = -0.2;
            }  
            //高度自适应降落的方法

            myDrone.setVelocityBody(vx, vy, vz, 0);
        }
        else
        {
            num_count_lost++;
            num_count = 0;

            //如果丢失计数超过阈值，则在不超过最大高度的前提下，无人机上升以获得更大的视野
            if(num_count_lost > Thres_vision_lost)
            {
                // cout << "lost the target, hold at current position!" << endl;
                if(myDrone.localPosition().pose.position.z <= fly_max_z)
                    myDrone.setVelocityBody(0, 0, 0.2, 0);
                else
                    myDrone.setVelocityBody(0, 0, 0, 0);
            }
        }

        //降落的2个条件：
        //1：水平距离小于阈值，即无人机抵达降落板中心位置附近
        //2：相对高度小于阈值，

        // // 如果相对距离小于阈值，计数增加
        // if(distance_pad < Thres_distance_land && det_result.success)
        // {
        //     num_count++;
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
        if(hight * distance_pad <= Thres_distance_land && num_count > Thres_count_land)
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
        if(myDrone.localPosition().pose.position.z <=  land_max_z)
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
        if(Flag_reach_pad_center && Flag_z_below_20cm)
        {
            // land and disarm!
            myDrone.land();
            sleep(3);

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
        ros::spinOnce();
        loop_rate.sleep();
    }
}