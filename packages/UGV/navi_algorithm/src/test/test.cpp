#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include <iostream>
#include <vector>  

int main(int argc, char **argv){
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    ros::Rate rate(10); 

    ros::Publisher rrt_tree_pub_;

    rrt_tree_pub_ = nh.advertise<visualization_msgs::Marker>("rrt_tree", 10);

    visualization_msgs::Marker line_;
    line_.header.frame_id="map";
    line_.header.stamp=ros::Time(0);
    line_.ns = "lines";
    line_.id = 1;
    line_.type=line_.LINE_LIST;
    line_.action=line_.ADD;
    line_.pose.orientation.w = 1.0;
    line_.scale.x = 0.1;
    line_.scale.y = 0.1;
    line_.color.r =255.0/255.0;
    line_.color.g= 0.0/255.0;
    line_.color.b =0.0/255.0;
    line_.color.a = 1.0;
    line_.lifetime = ros::Duration();

    geometry_msgs::Point p0;
    p0.x = 0.0;
    p0.y = 0.0;
    p0.z = 0.0;

    geometry_msgs::Point p1;
    p1.x = 1.0;
    p1.y = 1.0;
    p1.z = 1.0;

    geometry_msgs::Point p2;
    p2.x = -1.0;
    p2.y = 1.0;
    p2.z = 1.0;

    int a;
    std::cout << "input any to go on!!" << std::endl;
    a = std::cin.get();

    line_.points.push_back(p0);
    line_.points.push_back(p1);
    rrt_tree_pub_.publish(line_);

    line_.points.push_back(p0);
    line_.points.push_back(p2);
    rrt_tree_pub_.publish(line_);

    
    std::cout << "input any to go on 2!!" << std::endl;
    a = std::cin.get();

    // line_.header.frame_id="map";
    // line_.header.stamp=ros::Time(0);
    // line_.ns = "lines";
    // line_.id = 1;
    // line_.type=line_.LINE_LIST;
    // line_.action=line_.ADD;
    // line_.pose.orientation.w = 1.0;
    // line_.scale.x = 0.3;
    // line_.scale.y = 0.3;
    // line_.color.r =255.0/255.0;
    // line_.color.g= 0.0/255.0;
    // line_.color.b =0.0/255.0;
    // line_.color.a = 0.0;
    // line_.lifetime = ros::Duration();
    // line_.points.clear();
    // rrt_tree_pub_.publish(line_);

    // line_.id = 3;
    std::cout << "size :" << line_.points.size() << std::endl;
    line_.points.clear();

    std::cout << "size after :" << line_.points.size() << std::endl;

    rrt_tree_pub_.publish(line_);

    std::cout << "input any to go on 3!!" << std::endl;
    a = std::cin.get();



    line_.header.frame_id="map";
    line_.header.stamp=ros::Time(0);
    line_.ns = "lines";
    line_.id = 2;
    line_.type=line_.LINE_LIST;
    line_.action=line_.ADD;
    line_.pose.orientation.w = 1.0;
    line_.scale.x = 0.1;
    line_.scale.y = 0.1;
    line_.color.r =255.0/255.0;
    line_.color.g= 0.0/255.0;
    line_.color.b =0.0/255.0;
    line_.color.a = 1.0;
    line_.lifetime = ros::Duration();

    line_.points.push_back(p0);
    line_.points.push_back(p1);

    std::cout << "size after add:" << line_.points.size() << std::endl;

    rrt_tree_pub_.publish(line_);

    return 0;

}