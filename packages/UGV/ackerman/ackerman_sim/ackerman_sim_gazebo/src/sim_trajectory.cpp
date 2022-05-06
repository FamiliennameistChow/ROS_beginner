/*********************************************************************
 * 
 * 订阅： gazebo 插件的 /ground_truth/odom 里成绩信息
 * 发布： /sim_path/path  nav_msgs::Path 的 
 * 
**********************************************************************/

#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped vehicle_pos_;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    vehicle_pos_.header.stamp = ros::Time::now();
    vehicle_pos_.header.frame_id = msg->child_frame_id;
    vehicle_pos_.pose.position = msg->pose.pose.position;
    vehicle_pos_.pose.orientation = msg->pose.pose.orientation;
}

bool ifAddPose(geometry_msgs::PoseStamped _last_pose, geometry_msgs::PoseStamped _new_pose){
    return pow ( pow(_last_pose.pose.position.x-_new_pose.pose.position.x,2)
                +pow(_last_pose.pose.position.y-_new_pose.pose.position.y,2)
                +pow(_last_pose.pose.position.z-_new_pose.pose.position.z,2), 0.5) > 0.1;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "sim_trajectory");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    ros::Subscriber sim_odom_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/odom", 1, &odomCallback);
    ros::Publisher sim_path_pub = nh.advertise<nav_msgs::Path>("/sim_path/path",1);
    
    nav_msgs::Path path;
    path.header.frame_id = "map";       // 准确来说这个 frame 应该是里程计，暂时也没有

    geometry_msgs::PoseStamped last_vehicle_pos;
    last_vehicle_pos.header.frame_id    = "base_link";
    last_vehicle_pos.pose.position.x    = 0.0;
    last_vehicle_pos.pose.position.y    = 0.0;
    last_vehicle_pos.pose.position.z    = 0.0;
    last_vehicle_pos.pose.orientation.w = 1;
    last_vehicle_pos.pose.orientation.x = 0;
    last_vehicle_pos.pose.orientation.y = 0;
    last_vehicle_pos.pose.orientation.z = 0;


    int count = 0;
    while(ros::ok()){
        ros::spinOnce();
        if(ifAddPose(last_vehicle_pos, vehicle_pos_)){
            path.poses.push_back(vehicle_pos_);
            count++;
            if(count == 100){
                path.poses.erase(path.poses.begin());
                count = 0;
            }
        }
        sim_path_pub.publish(path);
        loop_rate.sleep();
    }
}
