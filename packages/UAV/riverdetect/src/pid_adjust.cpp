/**********************************************************/
/*
作者：阿杰 时间：2020.3.5
说明：利用位置四元数计算求得无人机在坐标系中实际偏航角，相对偏航角
     只用来调z方向角速度，x、y方向分速度由三角函数获得。
原理：以比例调节为主，借鉴了将金条拉成金丝进行逐步缩小直径拉丝的方
     法特点，通过对调节比例的多次放大，使得无人机在不同位置有不同
     的速度比例，该方法简单有效，但抗剧烈干扰能力不强，在大风情况
     下可能会产生剧烈抖动，不建议在室外实验。
*/
/**********************************************************/

//#include <iostream>
//#include <sstream>
//#include <math.h>
//#include <cmath>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <riverdetect/centerpoints.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
using namespace std;

//位置、速度调节比例参数设置。
float altitude = 25;
float V_a = 150;//angular speed
float V_lat = 0.3;//latitude speed
float V_long = 1.0;//longitude speed
float odometer;

//这个函数把飞机状态输出，后面等飞控、起飞都有用。
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

//位置信息获取。
geometry_msgs::PoseStamped local_position;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  local_position = *msg;
}

//图像处理后的返回值。
riverdetect::centerpoints point_result;
void centerpoints_cb(const riverdetect::centerpoints::ConstPtr& msg) {
    point_result = *msg;
}

//利用四元数获取坐标系中真实偏航角。
float true_angle() {
    //float siny_cosp;
    //float cosy_cosp;
    double siny_cosp = 2 * (local_position.pose.orientation.w * local_position.pose.orientation.z
                 + local_position.pose.orientation.x * local_position.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (local_position.pose.orientation.y * local_position.pose.orientation.y
                     + local_position.pose.orientation.z * local_position.pose.orientation.z);
    return atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_send");
    ros::NodeHandle n;

    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber centerpoints_sub = n.subscribe<riverdetect::centerpoints>("river_detect/centerpoints", 10, centerpoints_cb);
    ros::Subscriber local_position_sub = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_position_cb);
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Publisher velocity_pos_pub = n.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    ros::Rate rate(20);

    //这个循环应该是等飞控到位的。
    //wait for FCU connection.
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //设置位置，等会就飞到这么高。
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = local_position.pose.position.x;
    pose.pose.position.y = local_position.pose.position.y;
    pose.pose.position.z = local_position.pose.position.z + altitude;
    pose.pose.orientation.z = local_position.pose.orientation.z;
    pose.pose.orientation.w = local_position.pose.orientation.w;

    //这一堆应该是解锁用的，反正没有起飞不起来。
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    //飞起保持，等图片一来就飞。
    while(ros::ok() && !point_result.centersuccess){
        //飞机起飞所需的一堆判断，没有他飞机无法起飞。
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if(set_mode_client.call(offb_set_mode) &&offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");}
            last_request = ros::Time::now();}
        else {if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( arming_client.call(arm_cmd) &&arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");}
                last_request = ros::Time::now();}}
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }sleep(2.0);
/********************************************************/
//                    We Are READY!                     //
/********************************************************/
//这句设置velocity这个名称。
geometry_msgs::Twist velocity;
float angle;//相对偏航角/s
float theta;//偏航角
float error_x;//偏差
/*****************************************************/
//                        1st                        //
/*****************************************************/
//这个先让他把头磨正，等会飞的时候防止角度干扰。
int i = 0;
bool t = true;
while(ros::ok() && t && point_result.centersuccess)
{
    //相对偏航与中心偏差获取。
    angle = point_result.angle;
    error_x = point_result.error;

    //z方向角速度计入。
    velocity.angular.z = V_a * angle;//V_a = 150

    //偏航角获取。
    theta = true_angle();

    //机头横向速度计入(速度比例参数未放大，辅助一下)。
    velocity.linear.x = error_x * sin(theta) * V_lat/2;//V_lat = 0.3
    velocity.linear.y = error_x * -cos(theta) * V_lat/2;//V_lat = 0.3

    ROS_INFO("          .          ");
    ROS_INFO("        .....        ");
    ROS_INFO("      ...   ...      ");
    ROS_INFO("    ...       ...    ");
    ROS_INFO("  ...           ...  ");
    ROS_INFO("...               ...");
    ROS_INFO("angle = %lf\n",angle);

    velocity_pos_pub.publish(velocity);
    ros::spinOnce();
    rate.sleep();
    //设置一个判断，保证可以起飞跳出循环，不再受位置控制，进入下一步速度控制。
    ++i;
    if (i < 200) t = true;
    else t = false;
}
/*****************************************************/
//                        2nd                        //
/*****************************************************/
//在上一个调正的基础上，让偏差消失，飞机横向飞行。
i = 0;
t = true;
while(ros::ok() && t && point_result.centersuccess)
{
    //相对偏航与中心偏差获取。
    angle = point_result.angle;
    error_x = point_result.error;

    //设置偏差栅栏，当偏差大于0.03或0.08时，设置0.07的高墙，让横向速度增大，补偿无人机因惯性引起的非正常偏离。
    if (error_x > 0.03) {if (error_x > 0.08) error_x += 0.07;
        error_x += 0.07;}
    else if (error_x < -0.03) {if (error_x < -0.08) error_x -= 0.07;
        error_x -= 0.07;}

    //z方向角速度计入。
    velocity.angular.z = 5/3 * V_a * angle;//5/3 * V_a = 250

    //偏航角获取。
    theta = true_angle();

    //机头横向速度计入。
    velocity.linear.x = error_x * sin(theta) * 2 * V_lat;//V_lat = 2 * 0.3 = 0.6
    velocity.linear.y = error_x * -cos(theta) * 2 * V_lat;//V_lat = 2 * 0.3 = 0.6

    ROS_INFO("...   ...   ...    ");
    ROS_INFO("  ...   ...   ...  ");
    ROS_INFO("    ...   ...   ...");
    ROS_INFO("  ...   ...   ...  ");
    ROS_INFO("...   ...   ...    ");
    ROS_INFO("angle = %lf",angle);
    ROS_INFO("error_x = %lf",error_x);
    ROS_INFO("theta = %lf\n",theta);

    velocity_pos_pub.publish(velocity);
    ros::spinOnce();
    rate.sleep();
    //设置一个判断，保证可以起飞跳出循环，不再受位置控制，进入下一步速度控制。
    ++i;
    if (i < 600) t = true;
    else t = false;
}
/*****************************************************/
//                        3rd                        //
/*****************************************************/
//下面就控制飞机向前飞，不过飞的时候把前两步调节加在一起。
while(ros::ok() && point_result.centersuccess)
{
    //相对偏航与中心偏差获取。-
    angle = point_result.angle;
    error_x = point_result.error;

    //设置偏差栅栏，当偏差大于0.03或0.08时，设置0.07的高墙，让横向速度增大，补偿无人机因惯性引起的非正常偏离。
    if (error_x > 0.03) {if (error_x > 0.08) error_x += 0.07;
        error_x += 0.07;}
    else if (error_x < -0.03) {if (error_x < -0.08) error_x -= 0.07;
        error_x -= 0.07;}

    //z方向角速度计入。
    velocity.angular.z = 2 * V_a * angle;//2 * V_a = 300

    //偏航角获取。
    theta = true_angle();

    //机头横向速度计入。
    velocity.linear.x = error_x * sin(theta) * 5 * V_lat;//5 * 0.3 = 1.5
    velocity.linear.y = error_x * -cos(theta) * 5 * V_lat;//5 * 0.3 = 1.5
    velocity_pos_pub.publish(velocity);

    //机头向前的速度累加计入。
    velocity.linear.x += V_long * cos(theta);//V_long = 1.0
    velocity.linear.y += V_long * sin(theta);//V_long = 1.0

    ROS_INFO("          |          ");
    ROS_INFO("        |||||        ");
    ROS_INFO("      ||| | |||      ");
    ROS_INFO("    |||   |   |||    ");
    ROS_INFO("  |||   |||||   |||  ");
    ROS_INFO("|||   ||  |  ||   |||");
    ROS_INFO("angle = %lf",angle);
    ROS_INFO("error_x = %lf",error_x);
    ROS_INFO("theta = %lf\n",theta);

    velocity_pos_pub.publish(velocity);
    ros::spinOnce();
    rate.sleep();
}

//当point_result.centersuccess为false时，进入下面储存位置，不过好像从来没有false过。
pose.pose.position.x = local_position.pose.position.x;
pose.pose.position.y = local_position.pose.position.y;
pose.pose.position.z = local_position.pose.position.z;
pose.pose.orientation.z = local_position.pose.orientation.z;
pose.pose.orientation.w = local_position.pose.orientation.w;
while(ros::ok()){
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
}
    ros::spin();
    return 0;
}


