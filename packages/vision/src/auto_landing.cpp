/***************************************************************************************************************************
 * auto_landing.cpp
 *
 * Author: Danny
 *
 * Update Time: 2019.9.31
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

float kpx_land = 0.3,kpy_land = 0.3,kpz_land = 0.3; ////降落追踪控制算法的 速度 比例参数
float Max_velx_land = 1.0, Max_vely_land = 1.0, Max_velz_land = 0.5; //降落追踪控制算法的最大速度
float Thres_velx_land, Thres_vely_land, Thres_velz_land;   
float distance_pad = 100; //无人机与降落板中心的距离
float Thres_distance_land = 0.2; //允许降落最大距离阈值
int Thres_count_land = 30; //允许降落计数阈值
int num_count = 0; //允许降落计数
float fly_min_z = 0.2; //允许飞行最低高度[这个高度是指降落板上方的相对高度]
bool Flag_reach_pad_center = false; //是否到达目标点中心的标志位
bool Flag_z_below_20cm = false; //高度是否达到允许降落的标志位
float land_max_z = 0.2; //允许降落最大高度阈值
int num_count_lost = 0;  //视觉丢失计数
int num_count_z = 0; //降落过程中更改高度的计数阈值
int Thres_vision_lost = 30; //视觉丢失计数阈值
float vx, vy, vz;
chrono::steady_clock::time_point last_result_time;
bool Flag_first_result = true;
float x_begin, y_begin, z_begin, distance_pad_begin;

vision::detResult det_result;
void detResultCB(const vision::detResult::ConstPtr &msg)
{
    last_result_time = chrono::steady_clock::now();
    det_result = *msg;
    //计算与降落板之间的距离
    distance_pad = sqrt(det_result.x_err * det_result.x_err + det_result.y_err * det_result.y_err);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_landing");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    AeroDrone myDrone;
    
    ros::Subscriber detResult_sub = nh.subscribe("auto_landing/detResult", 100, detResultCB);

    geometry_msgs::PoseStamped setpoint;
    setpoint.pose.position.x = 0;
    setpoint.pose.position.y = 0;
    setpoint.pose.position.z = 5;

    myDrone.arm();
    sleep(3);
    // myDrone.takeoff();
    // sleep(5);
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
        // (chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - last_result_time)).count();
        if((chrono::duration_cast<chrono::duration<double>>(chrono::steady_clock::now() - last_result_time)).count() >= 1)
        {
            cout << "超过一秒钟未接收到识别结果，无人机将会悬停在当前位置" << endl;
            myDrone.setVelocityBody(0, 0, 0, 0);
            continue;
        }
        if(det_result.success)
        {
            if(Flag_first_result)
            {
                x_begin = det_result.x_err;
                y_begin = det_result.y_err;
                distance_pad_begin = distance_pad;
                z_begin = myDrone.localPosition().pose.position.z;
                Flag_first_result = false;
            }

            num_count_lost = 0;

            vx = abs(kpx_land * det_result.x_err) > Max_velx_land ?  (det_result.x_err > 0 ? Max_velx_land : -Max_velx_land) : (kpx_land * det_result.x_err);
            vy = abs(kpy_land * det_result.y_err) > Max_vely_land ?  (det_result.y_err > 0 ? Max_vely_land : -Max_vely_land) : (kpy_land * det_result.y_err);

            // 高度直接下降降落的方法
            // vz = abs(kpz_land * (myDrone.localPosition().pose.position.z - fly_min_z)) > Max_velz_land ?  -Max_velz_land : -(kpz_land * (myDrone.localPosition().pose.position.z  - fly_min_z));
            
            // 高度自适应降落的方法
            if(distance_pad > Thres_distance_land)
            {
                if(abs(distance_pad / distance_pad_begin - (myDrone.localPosition().pose.position.z - fly_min_z) / (z_begin - fly_min_z)) > 0.1)
                {
                    num_count_z++;
                    if(num_count_z > 30)
                    {
                        if(distance_pad / distance_pad_begin > (myDrone.localPosition().pose.position.z - fly_min_z) / z_begin){
                            cout << "无人机上升" << endl;
                            vz = abs(kpz_land * (myDrone.localPosition().pose.position.z - fly_min_z)) > Max_velz_land ?  Max_velz_land : (kpz_land * (myDrone.localPosition().pose.position.z  - fly_min_z));

                        }else{
                            cout << "无人机下降" << endl;
                            vz = abs(kpz_land * (myDrone.localPosition().pose.position.z - fly_min_z)) > Max_velz_land ?  -Max_velz_land : -(kpz_land * (myDrone.localPosition().pose.position.z  - fly_min_z));
                        }
                    }else{  
                        cout << "无人机保持高度" << endl;         
                        vz = 0;                
                    }
                }else{
                    cout << "无人机保持高度" << endl;
                    num_count_z = 0;
                    vz = 0;
                }
            }else{
                cout << "到达地标位置，无人机下降" << endl;
                vz = -0.1;
            }  
            //高度自适应降落的方法

            myDrone.setVelocityBody(vx, vy, vz, 0);
        }
        else
        {
            num_count_lost++;
            //如果丢失计数超过阈值，则在当前位置悬停
            if(num_count_lost > Thres_vision_lost)
            {
                cout << "lost the target, hold at current position!" << endl;
                myDrone.setVelocityBody(0, 0, 0, 0);
            }
        }

        //降落的2个条件：
        //1：水平距离小于阈值，即无人机抵达降落板中心位置附近
        //2：相对高度小于阈值，

        //如果相对距离小于阈值，计数增加
        if(distance_pad < Thres_distance_land && det_result.success)
        {
            num_count++;
            // cout<< "Distance_pad: " << distance_pad <<endl;
            // cout<< "Distance_land_count: " << num_count <<endl;
        }
        else
        {
            num_count = 0;
            // cout<< "Distance_pad: " << distance_pad <<endl;
            // cout<< "Distance_land_count: " << num_count <<endl;
        }

        //如果计数增加超过阈值，则满足降落的第一个条件（水平距离）
        if(distance_pad < Thres_distance_land && num_count > Thres_count_land)
        {
            cout<< "Flag_reach_pad_center: " << "true" <<endl;
            Flag_reach_pad_center = true;
        }
        else
        {
            cout<< "Flag_reach_pad_center: " << "flase" <<endl;
            Flag_reach_pad_center = false;
        }

        //如果 相对高度 小于 阈值，则满足降落的第二个条件（高度条件）
        if(myDrone.localPosition().pose.position.z <=  land_max_z)
        {
            cout<< "Flag_z_below_20cm: " << "true" <<endl;
            Flag_z_below_20cm = true;
        }
        else
        {
            cout<< "Flag_z_below_20cm: " << "flase" <<endl;
            Flag_z_below_20cm = false;
        }

        //如果降落的两个条件都满足，则降落并上锁
        if(Flag_reach_pad_center && Flag_z_below_20cm)
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
        ros::spinOnce();
        loop_rate.sleep();
    }
}