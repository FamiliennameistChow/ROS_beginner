/*********************************************************************
 * 
 * 订阅： 无
 * 发布： /waypoints  trajectory_msgs::MultiDOFJointTrajectory 需要跟踪的轨迹信息
 * 
**********************************************************************/

#include <cmath>
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class TestTrajectory{
public:
    TestTrajectory(ros::NodeHandle& nh);
    ~TestTrajectory(){};

    void pubSinPath();

private:
    // 这两个的区别就是路径的数据类型不同       
    ros::Publisher path_pub_;             // rviz 可以显示
    ros::NodeHandle n_;
    ros::Rate pub_rate_;

    nav_msgs::Path path_;
    geometry_msgs::PoseStamped waypoint_;

};

TestTrajectory::TestTrajectory(ros::NodeHandle& nh):n_(nh),pub_rate_(ros::Rate(1))
{
    path_.header.frame_id = "map";
    path_pub_ = n_.advertise<nav_msgs::Path>("/path_planned", 1);
}

void TestTrajectory::pubSinPath(){
    std::cout<<"A Sin() trajectory is doing pub!" <<std::endl;
    for(int i=0; i < 10000; i++){
        waypoint_.header.frame_id = "/map";
        waypoint_.pose.position.x = i*0.01;
        waypoint_.pose.position.y = 8*sin(M_PI_2 + i*2.0*M_PI/5000) - 8.0;
        waypoint_.pose.position.z = 0.5;
        waypoint_.pose.orientation.w = 1;
        waypoint_.pose.orientation.x = 0;
        waypoint_.pose.orientation.y = 0;
        waypoint_.pose.orientation.z = 0;
        path_.poses.push_back(waypoint_);
    }

    pub_rate_.sleep();
    path_pub_.publish(path_);
    pub_rate_.sleep();
    std::cout<<"[Pub END]: Sin() trajectory" <<std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "TestTrajectory");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    TestTrajectory test(nh);
    test.pubSinPath();
    ros::spin();
}
