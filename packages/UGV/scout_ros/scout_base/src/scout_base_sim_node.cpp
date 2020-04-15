#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "scout_sdk/scout_base.hpp"
#include "scout_base/scout_messenger.hpp"

using namespace wescore;

int main(int argc, char **argv)
{
    // setup ROS node
    ros::init(argc, argv, "scout_odom");
    ros::NodeHandle node(""), private_node("~");

    // instantiate a robot object
    ScoutBase robot;
    ScoutROSMessenger messenger(&robot, &node);

    // fetch parameters before connecting to robot
    std::string port_name;
    private_node.param<std::string>("port_name", port_name, std::string("can0"));
    private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom"));
    private_node.param<std::string>("base_frame", messenger.base_frame_, std::string("base_link"));
    private_node.param<bool>("simulated_robot", messenger.simulated_robot_, true);
    private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);

    // no connection for simulated robot
    // setup ROS subscription
    messenger.SetupSubscription();

    // publish robot state at 50Hz while listening to twist commands
    double linear, angular;
    ros::Rate rate_50hz(50); // 50Hz
    while (true)
    {
        messenger.GetCurrentMotionCmdForSim(linear, angular);
        messenger.PublishSimStateToROS(linear, angular);
        ros::spinOnce();
        rate_50hz.sleep();
    }

    return 0;
}