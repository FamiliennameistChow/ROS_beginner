//
//  main.cpp
//  Graduation
//
//  Created by NUC8i5BEH on 2020/3/17.
//  Copyright © 2020 NUC8i5BEH. All rights reserved.
//

#include "MoonLanding.hpp"
#include "AeroDrone.hpp"
#include <time.h>
#include <iostream>

using namespace cv;
using namespace std;

// 整个流程所用到的无人机的状态
namespace Status_n {
enum Status_t {
    INITING,
    RISING,
    FINGDING,
    MOVING,
    DETECTING,
    LANDING,
    LANDED,
};
}
typedef Status_n::Status_t Status;

Status current_status = Status_n::INITING;

// desc_type: fast, blob, surf, sift, orb, brisk, kaze, akaze, freak, daisy, brief
// match_tye: bf, knn
string desc_type = "surf";
string match_tye = "knn";

float takeoff_hight = 85;
int arrivedNum = 0;

int main(int argc, char **argv) {
    ros::init(argc, argv, "moon_landing_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(30);
    AeroDrone myDrone;
    

    MoonLanding moonLanding(nh, desc_type, match_tye);
    
     while(ros::ok())
    {
        switch(current_status)
        {
            case Status_n::INITING:
            {
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>initing<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                geometry_msgs::PoseStamped setpoint;
                setpoint.pose.position.x = -4;
                setpoint.pose.position.y = -4;
                setpoint.pose.position.z = takeoff_hight;
                

                // set to offboard mode 
                if (!myDrone.setMode("OFFBOARD"))
                {
                    ROS_ERROR("Fatal: Set OFFBOARD mode failed!");
                    break;
                }
                else
                {
                    ROS_INFO("Set OFFBOARD mode sent");
                }
                sleep(3);

                myDrone.arm();
                sleep(2);

                // publish a setpoint 
                myDrone.pubLocalPos(setpoint);
                current_status = Status_n::RISING;
                break;
            }
                


            case Status_n::RISING:
            {
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>rising<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                if(abs(myDrone.localPosition().pose.position.z - takeoff_hight) > 0.5)
                {
                    ROS_INFO("Still rising...");
                    sleep(2);
                    break;
                }

                ROS_INFO("Arrived the hight: %f", takeoff_hight);
                current_status = Status_n::FINGDING;
                break;
            }
                


            case Status_n::FINGDING:
            {
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>finding<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                cout << "请输入任意字符以开始自动降落" << endl;
                string input;
                cin >> input;
                moonLanding.searchLandArea();
                
                ROS_INFO("Have finded safe landing area");
                current_status = Status_n::MOVING;
                break;
            }
                


            case Status_n::MOVING:
            {
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>moving<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                if(moonLanding.trackingLandmark())
                {
                    if (abs(moonLanding.current_pose().x - moonLanding.landArea().center.x) + abs(moonLanding.current_pose().y - moonLanding.landArea().center.y) > 50)
                    {   
                        arrivedNum = 0;
                        ROS_INFO("Still tracking and moving...");
                        
                        float err_x, err_y, vel_x, vel_y;
                        err_x = moonLanding.current_pose().x - moonLanding.landArea().center.x;
                        err_y = moonLanding.current_pose().y - moonLanding.landArea().center.y;

                        if(err_x > 0) vel_y = 0.5;
                        else vel_y = -0.5;

                        // if(err_y > 0) vel_x = 0.5;
                        // else vel_x = -0.5;
                        if(err_y > 0) vel_x = abs(vel_y) * (err_y / err_x);
                        else vel_x = -abs(vel_y) * (err_y / err_x);

                        myDrone.setVelocityBody(vel_x, vel_y, -1, 0);

                        break;
                    }

                    if (abs(moonLanding.current_pose().x - moonLanding.landArea().center.x) + abs(moonLanding.current_pose().y - moonLanding.landArea().center.y) > 20)
                    {
                        arrivedNum = 0;
                        ROS_INFO("Still tracking and moving...");
                        
                        float err_x, err_y, vel_x, vel_y;
                        err_x = moonLanding.current_pose().x - moonLanding.landArea().center.x;
                        err_y = moonLanding.current_pose().y - moonLanding.landArea().center.y;

                        if(err_x > 0) vel_y = 0.2;
                        else vel_y = -0.2;

                        if(err_y > 0) vel_x = abs(vel_y) * (err_y / err_x);
                        else vel_x = -abs(vel_y) * (err_y / err_x);

                        myDrone.setVelocityBody(vel_x, vel_y, -0.5, 0);

                        break;
                    }

                    // 与降落区域距离小于一定的阈值则会计数增加，累计超过一定阈值则会认为到达目标区域，开始降落
                    arrivedNum++;
                    if(arrivedNum > 5)
                    {
                        current_status = Status_n::DETECTING;
                        break;
                    }

                }

                current_status = Status_n::DETECTING;
                break;
            }
                


            case Status_n::DETECTING:
            {
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>detecting<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;

                current_status = Status_n::LANDING;
                break;
            }



            case Status_n::LANDING:
            {
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>landing<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                cout << "无人机已经到达着陆点上方，准备着陆..." << endl;
                myDrone.land();

                current_status = Status_n::LANDED;
                break;
            }


            
            case Status_n::LANDED:
            {
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>landed<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
                if(myDrone.currentState().armed)
                {
                    cout << "无人机仍未上锁" << endl;
                    sleep(5);
                    // current_status = Status_n::LANDING;
                    break;
                }
                else
                {
                    cout << "无人机已经上锁，结束" << endl;
                    break;
                }
            }

        }
        rate.sleep();
    }

    return 0;
}