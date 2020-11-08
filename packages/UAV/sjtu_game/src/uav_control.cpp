
 /*
 * uav_control.cpp
 * 
 * Author： Born Chow
 * Date: 2020.10.14
 * 
 * 说明: 交大比赛无人机控制节点
 * 
 */

# include <drone_flight_modes.hpp>
# include <eigen3/Eigen/Dense>
#include <std_msgs/Bool.h>

# define DEBUG true;

enum missionState {
	INIT_MISSION, //任务初始化
	SEARCH, // 搜索号码牌
	CROSS_CIRCLE, //穿框
	LANDING, //降落
	ENDING
};

geometry_msgs::PoseStamped target_point;
geometry_msgs::Point center_point_in_body;
std_msgs::Bool landing_mark;

int c;

Eigen::MatrixXd waypoint = Eigen::MatrixXd::Zero(8, 7);
Eigen::MatrixXd circle_pos = Eigen::MatrixXd::Zero(8, 7);

void findCircleCallback(const geometry_msgs::Point::ConstPtr& msg){
    center_point_in_body = *msg;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "uav_control");
    ros::NodeHandle nh;

    AeroDrone myDrone("no_control");

    ros::Publisher point_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher landing_mark_pub = nh.advertise<std_msgs::Bool>("landing", 1);
    ros::Subscriber front_camera_reslut_sub = nh.subscribe<geometry_msgs::Point>("detect/circle_reslut_in_dody", 10, findCircleCallback);

    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !myDrone.currentState().connected) {
    ros::spinOnce();
    rate.sleep();
    }

    missionState  m_state= missionState::INIT_MISSION;
    ros::Time last_request = ros::Time::now();
    int comd_id = 0;
    int target_num = 1;

    //set waypoint 
    waypoint.row(0) << 0.0, 0.0, 1.6, 0.0, 0.0, 0.0, 1.0; //起飞点
    waypoint.row(1) << -1.0, 0.5, 1.2, 0.707, 0.0, 0.0, 0.707; //第一个框前
    waypoint.row(2) << -2.5, 3.2, 1.2, 0.0, 0.0, 0.0, 1.0; //第二个框前 4.2
    waypoint.row(3) << -1.5, 5.0, 1.2, 0.0, 0.0, 0.0, 1.0; //第三个框前 6.0
    waypoint.row(4) << 2.4, 7.5, 1.2, 0.0, 0.0, 0.0, 1.0; //第四个框前 8.5
    waypoint.row(5) << 1.8, 11.5, 1.2, 0.0, 0.0, 0.0, 1.0; //第五个框前 11.5
    waypoint.row(6) << -2.8, 16, 1.2, 0.0, 0.0, 0.0, 1.0; //第六个框前 16
    waypoint.row(7) << 0.0, 22, 1.2, 0.0, 0.0, 0.0, 1.0; //降落标志


    // test set circle pos
    circle_pos.row(1) << -1.0, 3.5, 1.2, 0.0, 0.0, 0.0, 1.0; //第一个框
    circle_pos.row(2) << -2.5, 6.2, 1.2, 0.0, 0.0, 0.0, 1.0; //第二个框前
    circle_pos.row(3) << -1.5, 8.0, 1.2, 0.0, 0.0, 0.0, 1.0; //第三个框前
    circle_pos.row(4) << 2.4, 10.5, 1.2, 0.0, 0.0, 0.0, 1.0; //第四个框前
    circle_pos.row(5) << 2, 14.5, 1.2, 0.0, 0.0, 0.0, 1.0; //第五个框前
    circle_pos.row(6) << -2.8, 19, 1.2, 0.0, 0.0, 0.0, 1.0; //第六个框前

    while (ros::ok())
    {
        comd_id++;
        switch (m_state)
        {
            case INIT_MISSION:
            {
                std::cout << "==========[mission init]==========" << std::endl;
                if (!myDrone.currentState().armed)
                {
                    ROS_INFO("Ready to Takeoff");
                    if (myDrone.arm())
                    {
                        ROS_INFO("Vehicle armed");
                    }
                    // myDrone.takeoff();
                    // ros::Time last_request = ros::Time::now();
                    target_point.header.seq = comd_id;
                    target_point.header.stamp = ros::Time::now();
                    target_point.header.frame_id = "world";
                    
                    target_point.pose.position.x = 0.0;
                    target_point.pose.position.y = 0.0;
                    target_point.pose.position.z = 1.6;
                    target_point.pose.orientation.x = 0.0;
                    target_point.pose.orientation.y = 0.0;
                    target_point.pose.orientation.z = 0.646;
                    target_point.pose.orientation.w = 0.707;
                    point_pub.publish(target_point);
                    sleep(3);
                }

                if (myDrone.currentState().mode != "OFFBOARD"){
                    if (myDrone.setMode("OFFBOARD")) {
                        ROS_INFO("Offboard Mode enabled");
                    }
                }

                m_state = SEARCH;
                
                break;
            }

            case SEARCH:
            {
                std::cout << "==========[mission SEARCH]==========" <<  target_num << std::endl;
                
                target_point.header.seq = comd_id;
                target_point.header.stamp = ros::Time::now();
                target_point.header.frame_id = "world";
                
                target_point.pose.position.x = waypoint.row(target_num)(0);
                target_point.pose.position.y = waypoint.row(target_num)(1);
                target_point.pose.position.z = waypoint.row(target_num)(2);
                target_point.pose.orientation.x = waypoint.row(target_num)(3);
                target_point.pose.orientation.y = waypoint.row(target_num)(4);
                target_point.pose.orientation.z = waypoint.row(target_num)(5);
                target_point.pose.orientation.w = waypoint.row(target_num)(6);

                #ifdef DEBUG
                cout<< "[uav control] where I want to go IS: " <<  
                target_point.pose.position.x  << " " << 
                target_point.pose.position.y  << " " << 
                target_point.pose.position.z  << endl;

                cout<< "Plz input 1 to [GO ON]" << endl;
                cin >> c;
                #endif

                point_pub.publish(target_point);
                sleep(8);

                if (target_num <= 6)
                {
                    m_state = CROSS_CIRCLE;
                }else
                {
                    m_state = LANDING;
                }

                break;
            }

            case CROSS_CIRCLE:
            {
                std::cout << "==========[mission CROSS_CIRCLE]==========" << target_num << std::endl;
                
            
                if(center_point_in_body.x > 10 && center_point_in_body.x < 0) //说明检测无效
                {
                    continue;
                }
                //
                cout << "reslut: " << center_point_in_body.x <<" " <<center_point_in_body.y << " " << center_point_in_body.z << endl;

                target_point = myDrone.localPosition();

                target_point.header.seq = comd_id;
                target_point.header.stamp = ros::Time::now();
                target_point.header.frame_id = "world";
                
                cout << "uav pose: " << target_point.pose.position.x << " " << target_point.pose.position.y  << " " << target_point.pose.position.z << " "
                << target_point.pose.orientation.x << " " <<target_point.pose.orientation.y  <<" "<<target_point.pose.orientation.z<<" "<<target_point.pose.orientation.w<<endl;
                target_point.pose.position.x = target_point.pose.position.x - center_point_in_body.y;
                target_point.pose.position.y = target_point.pose.position.y + center_point_in_body.x + 1.0;
                target_point.pose.position.z = target_point.pose.position.z + center_point_in_body.z;
                target_point.pose.orientation.x = circle_pos.row(target_num)(3);
                target_point.pose.orientation.y = circle_pos.row(target_num)(4);
                target_point.pose.orientation.z = circle_pos.row(target_num)(5);
                target_point.pose.orientation.w = circle_pos.row(target_num)(6);

                #ifdef DEBUG
                cout<< "[uav control] where I want to go IS: " <<  
                target_point.pose.position.x  << " " << 
                target_point.pose.position.y  << " " << 
                target_point.pose.position.z  << endl;

                cout<< "Plz input 1 to [GO ON]" << endl;
                cin >> c;
                #endif

                point_pub.publish(target_point);
                sleep(3);

                target_num++;
                m_state = SEARCH;
                // if (target_num <= 6)
                // {
                //     m_state = SEARCH;
                // }else
                // {
                //     m_state = LANDING;
                // }
                break;
            }

            case LANDING:
            {   
                std::cout << "==========[mission LANDING]==========" << target_num << std::endl;
                //
                // add code to find land target
                //

                
                landing_mark.data = true;
                landing_mark_pub.publish(landing_mark);

                myDrone.land();

                break;
            }

            default:
                break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    

}