/**
 * @file    drone_control.cpp
 * @author  Kumaron
 * @date    2019.04.22
 * @brief   drone control node
 *          written with mavros 0.26.3, mavlink 1.0.10, px4 flight v1.8.2
 *          1. Takeoff in mission mode and execute waypoint mission
 *          2. Receive data from water monitoring node 
 */

//////////////////////////////////////////////////////////
//
// Includes
//
//////////////////////////////////////////////////////////
#include <ros/ros.h>
// ros msgs includes
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointPush.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
// ros serial include
#include <serial/serial.h>
// boost includes for parsing QGC plan file (JSON)
#include <boost/bind.hpp>
#include <boost/cstdfloat.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <exception>
#include <iostream>
#include <set>
#include <string>
#include <inttypes.h>

//////////////////////////////////////////////////////////
//
// Namespace and global variables
//
//////////////////////////////////////////////////////////
// For parsing QGC waypoint plan (JSON)
namespace bpt = boost::property_tree;

/**
 * global variables and flags
 */
bool g_is_home_gps_set = false;

//////////////////////////////////////////////////////////
//
// Parse QGC plan 
//
//////////////////////////////////////////////////////////
/**
 * @function        getWaypointFromQGCPlan
 * @parameters      
 * @return          void
 * @description     
 */
void getWaypointFromQGCPlan(const std::string& qgc_plan_file, mavros_msgs::WaypointPush* wp_list)
{
    try
    {
        std::ifstream file(qgc_plan_file);
        std::stringstream ss;

        ss << file.rdbuf();
        file.close();

        // Parse QGC plan begins
        //////////////////////////////////////////////////////////
        bpt::ptree mission_pt;
        bpt::read_json(ss, mission_pt);

        // NOTE: Unexpected type while reading values will cause an exception.
        bool first = true;
        // FOREACH mission item in the list
        for(auto& mi : mission_pt.get_child("mission.items"))
        {
            mavros_msgs::Waypoint wp{};
            // we have mission items now
            wp.frame = mi.second.get<int>("frame");
            wp.command = mi.second.get<int>("command");
            wp.autocontinue = mi.second.get<bool>("autoContinue");
            // only 1st mission item should be set to true.
            wp.is_current = first ? true : false;
            first = false;
            // parameters
            std::vector<double> params;
            for(auto& p : mi.second.get_child("params"))
            {
                params.push_back(p.second.get<double>(""));
            }
            wp.param1 = params[0];
            wp.param2 = params[1];
            wp.param3 = params[2];
            wp.param4 = params[3];
            wp.x_lat = params[4];
            wp.y_long = params[5];
            wp.z_alt = params[6];
            // add it to waypoint list
            wp_list->request.waypoints.push_back(wp);
        }        
        //////////////////////////////////////////////////////////
        // Parse QGC plan ends     
    }
    catch(std::exception const& e)
    {
        ROS_ERROR("%s", e.what());
        throw;
    }
}


//////////////////////////////////////////////////////////
//
// Callbacks
//
//////////////////////////////////////////////////////////
/**
 * @function        stateCallback
 * @parameters      
 * @return          void
 * @description    
 */
void stateCallback(const mavros_msgs::State::ConstPtr& msg, mavros_msgs::State* current_state){
    *current_state = *msg;
}

/**
 * @function        setHomeGPSCallback
 * @parameters      
 * @return          void
 * @description    
 */
void setHomeGPSCallback(const sensor_msgs::NavSatFixConstPtr& msg, sensor_msgs::NavSatFix* home_gps)
{
    *home_gps = *msg;
    g_is_home_gps_set = true;
    ROS_INFO("Received Home: %lf, %lf, %lf", home_gps->latitude, home_gps->longitude, home_gps->altitude);
}

/**
 * @function        getGpsPositionCallback
 * @parameters      
 * @return          void
 * @description    
 */
void getGpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& msg, sensor_msgs::NavSatFix* gps_pos)
{
    *gps_pos = *msg;
}


//////////////////////////////////////////////////////////
//
// Main
//
//////////////////////////////////////////////////////////
/**
 * @function        main
 * @parameters      
 * @return          
 * @description     
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_control");
    ros::NodeHandle nh;

    // msgs variables
    mavros_msgs::State current_state{};
    mavros_msgs::WaypointPush wp_list{};
    sensor_msgs::NavSatFix home_gps{};
    sensor_msgs::NavSatFix gps_pos{};
    mavros_msgs::SetMode set_mode;
    mavros_msgs::CommandBool arm_cmd;

    // subscriber and publisher
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, boost::bind(stateCallback, _1, &current_state));
    ros::Subscriber home_gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, boost::bind(setHomeGPSCallback, _1, &home_gps));
    ros::Subscriber global_gps_pub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, boost::bind(getGpsPositionCallback, _1, &gps_pos));
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>
            ("mavros/mission/push");

    // load mission items from QGC plan
    try
    {
        getWaypointFromQGCPlan(argv[1], &wp_list);
    }
    catch(std::exception const& e)
    {
        ROS_ERROR("Error in loading waypoints from the file %s.", argv[1]);
        abort();
    }

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ROS_INFO("Waiting for Home to be set");
    while(ros::ok() && !g_is_home_gps_set)
    {
        ros::spinOnce();
        rate.sleep();
    }
    // don't need FCU home position now
    home_gps_sub.shutdown();

    // send wps to vehicle
    while(ros::ok())
    {
        if(wp_client.call(wp_list))
        {
            if(!wp_list.response.success)
            {
                // wait till succeed in sending wps
                ROS_INFO("Waiting for succeed in sending waypoints");
                ros::spinOnce();
                rate.sleep();
            }
            else
            {
                ROS_INFO("Waypoints sent to vehicle");
                break;
            }
        }
    }
    
    // set to Mission mode
    ROS_INFO("Now, setting to Mission mode ...");
    set_mode.request.custom_mode = "AUTO.MISSION";
    if(set_mode_client.call(set_mode) && set_mode.response.mode_sent)
    {
        ROS_INFO("In Mission mode now");
        // arm
        arm_cmd.request.value = true;
        if(!current_state.armed)
        {
            if(arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed ...");
            }
        }
        else
        {
            ROS_ERROR("Failed to set Mission mode!");
            ros::shutdown();
        }    
    }

    // not overload FCU
    ROS_INFO("Pause for 5 secs to keep vehicle at ease");
    sleep(5.0);
    ROS_INFO("Resumed. Missions in execution ...");
    bool is_armed = current_state.armed;

    // wait for FCU connection
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        is_armed = current_state.armed;
        if(!is_armed)
            break;
    }
    ROS_INFO("Mission accomplished!");

    return EXIT_SUCCESS;
}

