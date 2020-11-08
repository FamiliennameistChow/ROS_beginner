#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include "quadrotor_msgs/PositionCommand.h"


ros::Publisher command_pub;

mavros_msgs::PositionTarget pose;

void commandCB(const quadrotor_msgs::PositionCommand::ConstPtr& msg){
    pose.header.seq = msg->header.seq;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = msg->header.frame_id;
    pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    pose.type_mask = mavros_msgs::PositionTarget::FORCE;


    
    pose.position.x = msg->position.x;
    pose.position.y = msg->position.y;
    pose.position.z = msg->position.z;

    pose.velocity.x = msg->velocity.x;
    pose.velocity.y = msg->velocity.y;
    pose.velocity.z = msg->velocity.z;

    pose.acceleration_or_force.x = msg->acceleration.x;
    pose.acceleration_or_force.y = msg->acceleration.y;
    pose.acceleration_or_force.z = msg->acceleration.z;

    pose.yaw = msg->yaw;
    pose.yaw_rate = msg->yaw_dot;

    if (msg->velocity.x == 0 && msg->velocity.y == 0 && msg->velocity.z == 0 && 
        msg->acceleration.x == 0 && msg->acceleration.y == 0 && msg->acceleration.z == 0)
    {
        pose.yaw = 1.57;
    }

    command_pub.publish(pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "command_trans");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);

    ros::Subscriber command_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
            ("/planning/pos_cmd", 10, commandCB);
    command_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);
    
    while(ros::ok())
    {
        ROS_INFO("in the loop");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
