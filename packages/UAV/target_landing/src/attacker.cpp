 /********************************************************
 * attacker.cpp
 * attacker in mulit-UAV
 * 
 * Author： Born Chow
 * Date: 2021.03.30
 * 
 * 说明：多机任务分配，这里实现攻击机的代码－－attacker类　
 * 
 * 【订阅】
 * 
 * 【发布】
 * 
 ******************************************************/

#include "ros/ros.h"
#include <mavros_msgs/GlobalPositionTarget.h>
#include "target_landing/AttackerMsg.h"
#include "target_landing/scout_plan_msg.h"
#include "drone_flight_modes.hpp"
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/WaypointList.h>

#include <iostream>
#include <math.h>

using namespace std;


enum UAVState {
	INIT_MISSION, //任务初始化
	BIDDING, // 投标
	CONFIRM, //确认
	WINBID, //执行,中标
    BACK_HOME,
	ENDING
};

#define PI 3.14

// class start
//-----------------------------------------------------------------------------------------------//

class Attacker
{
private:
    ros::NodeHandle n_;

    // ros消息发布与订阅
    ros::Publisher bidding_cost_pub_; // 发布投标cost
    ros::Publisher bidding_exec_result_pub_; //发布执行结果

    ros::Subscriber bidding_msg_sub_; // 订阅招标
    ros::Subscriber bidding_confirm_sub_; //订阅中标信息
    ros::Subscriber mission_list_sub_; //订阅正在执行的任务列表


    ros::ServiceClient wp_client;
    ros::ServiceClient wp_clear_client;
    ros::Rate rate_ = ros::Rate(40.0);

    // ros 参数
    int uav_num_;
    bool state_change_ ; //test
    bool is_me_; //确定中标标志
    int order_num_; //确认中标后, 距离目标的cost排序

    AeroDrone* my_uav_; 

    // 消息参数
    target_landing::scout_plan_msg biding_target_;
    target_landing::scout_plan_msg biding_comfirm_;
    target_landing::AttackerMsg biding_cost_;


    // 无人机参数
    UAVState uav_state_;
    sensor_msgs::NavSatFix uav_gps_;
    sensor_msgs::NavSatFix uav_home_gps_;
    sensor_msgs::NavSatFix target_gps_;
    // sensor_msgs::NavSatFix target_gps_last_; //记录上一次任务的值
    mavros_msgs::WaypointList mission_perform_list_; //正在执行的任务列表
    // vector<sensor_msgs::NavSatFix> mission_perform_list_;  //正在执行的任务列表

    double EARTH_RADIUS = 6378137;  //地球半径/米

private:
    void bidding_msg_callback(const target_landing::scout_plan_msg::ConstPtr &msg);
    void bidding_confirm_callback(const target_landing::scout_plan_msg::ConstPtr &msg);
    void mission_perform_callback(const mavros_msgs::WaypointList::ConstPtr &msg);
    double comput_dis(sensor_msgs::NavSatFix p1, sensor_msgs::NavSatFix p2);
    double rad(double d) {
        return d * PI / 180.0;
    } //deg2rad

    void go_waypoint(mavros_msgs::Waypoint wp);
    bool target_in_mission(target_landing::scout_plan_msg target, mavros_msgs::WaypointList mission_list);
    bool target_in_mission(target_landing::scout_plan_msg target, vector<sensor_msgs::NavSatFix> mission_list); //no use

public:
    Attacker(ros::NodeHandle& nh, ros::NodeHandle &private_nh);
    void run();
    ~Attacker();
};

//-----------------------------------声明---------------------------------------------------------------//

Attacker::Attacker(ros::NodeHandle& nh, ros::NodeHandle &private_nh):
n_(nh),
is_me_(false),
state_change_(false)
{
    // 读取参数服务器
    private_nh.param<int>("uav_num", uav_num_, 2);


    string topic_head = "/uav" + to_string(uav_num_);


    // 定义发布与订阅
    bidding_cost_pub_ = n_.advertise<target_landing::AttackerMsg>("attacker_cost", 10);
    bidding_exec_result_pub_ = n_.advertise<target_landing::AttackerMsg>("attacker_reuslt", 10);
    bidding_msg_sub_ = n_.subscribe<target_landing::scout_plan_msg>("scout_biding_msg", 1, &Attacker::bidding_msg_callback, this);
    bidding_confirm_sub_ = n_.subscribe<target_landing::scout_plan_msg>("scout_biding_confirm_msg", 1, &Attacker::bidding_confirm_callback, this);
    mission_list_sub_ = n_.subscribe<mavros_msgs::WaypointList>("mission_perform_list", 1, &Attacker::mission_perform_callback, this);


    wp_client = n_.serviceClient<mavros_msgs::WaypointPush>(topic_head + "/mavros/mission/push");
    wp_clear_client = n_.serviceClient<mavros_msgs::WaypointClear>(topic_head + "/mavros/mission/clear");


    //初始化参数
    uav_state_ = INIT_MISSION;
    my_uav_ = new AeroDrone(uav_num_);
    // latitude_last_ = 0;

    // run();

}


void Attacker::bidding_msg_callback(const target_landing::scout_plan_msg::ConstPtr &msg){
    
    biding_target_ = *msg;
}

void Attacker::mission_perform_callback(const mavros_msgs::WaypointList::ConstPtr &msg){
    mission_perform_list_ = *msg;
}


void Attacker::bidding_confirm_callback(const target_landing::scout_plan_msg::ConstPtr &msg){
    biding_comfirm_ = *msg;
}

double Attacker::comput_dis(sensor_msgs::NavSatFix p1, sensor_msgs::NavSatFix p2){
    // https://blog.csdn.net/weixin_40437029/article/details/107784778
    // 计算两个GPS坐标的距离
    double radLat1 = rad(p1.latitude);
    double radLat2 = rad(p2.latitude);
    double radLong1 = rad(p1.longitude);
    double radLong2 = rad(p2.longitude);

    double a = radLat1 - radLat2;
    double b = radLong1 - radLong2;
    double s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
    s = s * EARTH_RADIUS;
    s = round(s * 10000) / 10000;
}

void Attacker::run(){
    
    switch (uav_state_)
    {
        case INIT_MISSION:
        {
            if (!my_uav_->currentState().armed)
            {
                ROS_INFO("Ready to Takeoff");
                if (my_uav_->arm())
                {
                    ROS_INFO("Vehicle armed");
                }
                my_uav_->takeoff();
                sleep(5);
                if(my_uav_->setMode("OFFBOARD")){
                    ROS_INFO("Offboard Mode enabled");
                }
            }
            ROS_INFO("\033[1;32m--uav %d-->\033[0m uav ready.", uav_num_);
            uav_home_gps_ = my_uav_->globalPosition();
            cout << "my home gps: " << uav_home_gps_ << endl;
            uav_state_ = BIDDING;// BIDDING;
            state_change_ = true;
            break;
        }


        case BIDDING:
        {
            if (state_change_)
            {
                ROS_INFO("\033[1;32m--uav %d-->\033[0m waiting target.", uav_num_);
                
                state_change_ = false;
            }

            // cout <<" mission_list_: " <<  mission_perform_list_.size() << endl;

            uav_gps_ = my_uav_->globalPosition();
            // cout << "my   gps : "   <<uav_gps_.latitude << " || " << uav_gps_.longitude << " || " <<uav_gps_.altitude << endl;
            // cout << "targ gps : "  
            //      << biding_target_.target_GPS.latitude  << " || " 
            //      << biding_target_.target_GPS.longitude << " || " 
            //      << biding_target_.target_GPS.altitude  <<endl;

            // if (biding_target_.target_GPS.latitude != 0 && comput_dis(biding_target_.target_GPS, target_gps_last_) > 0.5)  //消息有效
            if(!target_in_mission(biding_target_, mission_perform_list_) && biding_target_.target_GPS.latitude != 0) //消息有效
            {
                // target_gps_last_ = biding_target_.target_GPS;
                cout << "my   gps : "   <<uav_gps_.latitude << " || " << uav_gps_.longitude << " || " <<uav_gps_.altitude << endl;
                cout << "targ gps : "  
                     << biding_target_.target_GPS.latitude  << " || " 
                     << biding_target_.target_GPS.longitude << " || " 
                     << biding_target_.target_GPS.altitude  <<endl;

                target_gps_ = biding_target_.target_GPS;

                // mission_perform_list_.push_back(target_gps_);
                
                // biding_target_.target_GPS.latitude = 0;

                double cost = comput_dis(uav_gps_, target_gps_);
                cout << "cost: " << cost << endl;

                biding_cost_.cost = cost;
                biding_cost_.status = 1;
                biding_cost_.UAV_NUM = uav_num_;
                bidding_cost_pub_.publish(biding_cost_);
                uav_state_ = CONFIRM; //CONFIRM;
                state_change_ = true;
                break;
            }
            // 消息无效,等待
            break;
            
        }

        case CONFIRM:
        {
            if (state_change_)
            {
                ROS_INFO("\033[1;32m--uav %d-->\033[0m confirm target.", uav_num_);
                state_change_ = false;
            }

            if (!biding_comfirm_.UAV_NUM.empty() && biding_comfirm_.UAV_NUM.size() == biding_target_.num_val) // 消息有效
            {
                is_me_ = false;
                cout <<"win bid num:" << biding_comfirm_.UAV_NUM.size() << endl;
                for(int i = 0; i < biding_comfirm_.UAV_NUM.size(); ++i)
                {
                    int a = biding_comfirm_.UAV_NUM[i];
                    cout << "confirm num: " <<  a  << endl;
                    cout << "my  num: " <<  uav_num_  << " is me : " << (a == uav_num_) << endl;
                    if (a == uav_num_) // 确认中标
                    {
                        is_me_ = true;
                        order_num_ = i;
                        break;
                    }  
                }

                biding_comfirm_.UAV_NUM.resize(0);

                if (is_me_) 
                {
                    // 确认中标,进入执行状态
                    uav_state_ = WINBID;
                    state_change_ = true;
                    break;
                }else
                {
                    // 未中标,返回投标状态
                    uav_state_ = BIDDING;
                    state_change_ = true;
                    break;
                }
            }
            // 消息无效,等待
            break;
        }

        case WINBID:
        {
            if (state_change_)
            {
                ROS_INFO("\033[1;32m--uav %d-->\033[0m win bid.", uav_num_);
                state_change_ = false;
            }
            

            mavros_msgs::Waypoint wp;
            
            // see MISSION_ITEM ( #39 ) in https://mavlink.io/en/messages/common.html#MISSION_ITEM
            // see MAV_FRAME https://mavlink.io/en/messages/common.html#MAV_FRAME
            wp.frame = 3;  
            // Navigate to waypoint 
            // see  MAVLink Commands (MAV_CMD) https://mavlink.io/en/messages/common.html#MAV_CMD
            wp.command = 22;  
            wp.is_current = false;
            wp.autocontinue = false;
            wp.param1 = 5.0;
            wp.param2 = 0.0;
            wp.param3 = 0.0;
            wp.param4 = 0.0;
            wp.x_lat  = (order_num_% 2) ? target_gps_.latitude - 0.00001 * (order_num_% 2) : target_gps_.latitude + 0.00001 *(order_num_% 2);
            wp.y_long = (order_num_% 2) ? target_gps_.longitude - 0.00001 * (order_num_% 2) : target_gps_.longitude + 0.00001 * (order_num_% 2);
            wp.z_alt  = 10.0 + uav_num_*0.9;

            sensor_msgs::NavSatFix target_new_gps;
            target_new_gps.latitude = wp.x_lat;
            target_new_gps.longitude = wp.y_long;


            if (!my_uav_->currentState().armed)
            {
                if (my_uav_->arm())
                {
                    ROS_INFO("--uav %d--> Vehicle armed", uav_num_);
                }
            }
            
            go_waypoint(wp);
            sleep(5);

            float dis = 100000;
            while (dis > 0.5)   // dis <0.5 reached 
            {
                uav_gps_ = my_uav_->globalPosition();
                dis = comput_dis(uav_gps_, target_new_gps);
                // cout << "dis: " << dis <<  endl;
                ROS_INFO("\033[1;32m--uav %d-->\033[0m dis : %f.", uav_num_, dis);
            }

            //
            // code when reached
            //
            sleep(5);

            uav_state_ = BACK_HOME;
            state_change_ = true;
            break;
            

        }

        case BACK_HOME:
        {
            if (state_change_)
            {
                ROS_INFO("\033[1;32m--uav %d-->\033[0m back home.", uav_num_);
                state_change_ = false;
            }
            

            mavros_msgs::Waypoint wp;
            // see MISSION_ITEM ( #39 ) in https://mavlink.io/en/messages/common.html#MISSION_ITEM
            // see MAV_FRAME https://mavlink.io/en/messages/common.html#MAV_FRAME
            wp.frame = 3;  
            // Navigate to waypoint 
            // see  MAVLink Commands (MAV_CMD) https://mavlink.io/en/messages/common.html#MAV_CMD
            wp.command = 21;  
            wp.is_current = false;
            wp.autocontinue = false;
            wp.param1 = 0.0;
            wp.param2 = 0.0;
            wp.param3 = 0.0;
            wp.param4 = 0.0;
            wp.x_lat  = uav_home_gps_.latitude;
            wp.y_long = uav_home_gps_.longitude;
            wp.z_alt  = 10.0 + uav_num_*0.9;

            go_waypoint(wp);
            sleep(5);

            // my_uav_->setMode("AUTO.RTL");

            if (my_uav_->currentState().armed == false)
            { 
                uav_state_ = BIDDING;
                state_change_ = true;
                break;
            }
            break;
        }

        case ENDING:
        {
            /* code */
            break; 
        }

        default:
            break;
    }
}

bool Attacker::target_in_mission(target_landing::scout_plan_msg target, vector<sensor_msgs::NavSatFix> mission_list){
    if (mission_list.empty())
    {
        return false;
    }

    for (auto i : mission_list)
    {
        if (comput_dis(i, target.target_GPS) < 0.2)
        {
            return true;
        }
    }
}

bool Attacker::target_in_mission(target_landing::scout_plan_msg target, mavros_msgs::WaypointList mission_list){
    
    if (mission_list.waypoints.size() == 0)
    {
        return false;
    }
    

    for(int i=0; i< mission_list.waypoints.size(); i++){
        sensor_msgs::NavSatFix p_m;
        p_m.latitude = mission_list.waypoints[i].x_lat;
        p_m.longitude = mission_list.waypoints[i].y_long;

        if (comput_dis(p_m, target.target_GPS) < 0.2)
        {
            return true;
        }
    }

    return false;
}

void Attacker::go_waypoint(mavros_msgs::Waypoint wp){
    mavros_msgs::WaypointPush wp_list{};

    wp_list.request.start_index = 0;
    wp_list.request.waypoints.push_back(wp);

    mavros_msgs::WaypointClear srv_wpclear;
    while (ros::ok())
    {
        if (wp_clear_client.call(srv_wpclear) && srv_wpclear.response.success)
        {
            ROS_INFO("the previous flight plan has been cleared");
            break;
        }
        else
        {
            ROS_INFO("clearing the previous flight plan...");
            ros::spinOnce();
            rate_.sleep();
        }
    }

    ROS_INFO("Now, Sending WPs to Vehicle...");
    while (ros::ok())
    {
        if (wp_client.call(wp_list))
        {
            if (!wp_list.response.success)
            {
                // Lets wait till we succeed in sending WPs.
                ROS_ERROR("Lets wait till we succeed in sending WPs");
                ros::spinOnce();
                rate_.sleep();
            }
            else
            {
                ROS_INFO("WPs sent to Vehicle");
                my_uav_->setMode("AUTO.MISSION");
                break;
            }
        }
    }
}

Attacker::~Attacker()
{

}

//  class Attacker end
//-----------------------------------------------------------------------------------------------//

int main(int argc, char **argv)
{

    ros::init(argc, argv, "attacker");
    ROS_INFO("\033[1;32m---->\033[0m attacker Node Started.");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate rate(30.0);

    Attacker attacker(nh, nh_);

    while(ros::ok())
    {
        attacker.run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}