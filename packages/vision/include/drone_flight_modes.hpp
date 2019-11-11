/*
* @file  : drone_flight_modes.hpp
* @brief : lib for drone control
* @date  : 2019.11.2
* @author: Danny
*
*/
#pragma once

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointReached.h>
#include <sensor_msgs/NavSatFix.h>
#include <cmath>

// Boost includes for parsing QGC plan file (JSON)
#include <boost/bind.hpp>
#include <boost/cstdfloat.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <exception>
#include <iostream>
#include <set>
#include <string>

using namespace std;

// For parsing QGC Waypoint plan (JSON)
namespace bpt = boost::property_tree;

class AeroDrone
{
public:
  AeroDrone();
  ~AeroDrone();

  // Actions
  bool arm();
  bool disarm();
  bool takeoff();
  bool land();

  bool setMode(string modeName); // 设置无人机模式
  mavros_msgs::State currentState(); // 获取无人机的当前状态
  geometry_msgs::PoseStamped localPosition(); // 获取无人机的local_positon
  void pubLocalPos(geometry_msgs::PoseStamped setpoint); // 发布local positon的设定值

  void uploadMission(const std::string& qgc_plan_file); // 上传航点任务
  mavros_msgs::WaypointList waypoints(); // 获取机内航点任务
  int waypointReached(); // 获取已到达的航点数量（takeoff点和return点不计入）

  void moveBody(float x, float y, float z); // 按照机体坐标系移动
  void moveENU(float x, float y, float z); // 按照ENU坐标系移动（在当前位置坐标的基础上）
  void moveENUto(float x, float y, float z); // 将无人机移动到ENU坐标系下指定位置
  void rotateAngle(double degree); //让无人机偏航指定的角度（在当前偏航角的基础上）
  void rotateAngleto(double degree); //设定无人机偏航角为指定角度
  double yawAngle(); // 无人机当前的偏航角


private:
  void resetHome();
  void getHomeGeoPoint();
  void setHomeGeoPointCB(const mavros_msgs::HomePositionConstPtr& home);
  void watchFlightModeThread();
  void getFlightModeCB(const mavros_msgs::StateConstPtr& state);

  ros::NodeHandle nh_;
  mavros_msgs::HomePosition home_{};
  bool home_set_ = false;
  ros::Rate rate_ = ros::Rate(20.0);
  boost::thread* thread_watch_flight_mode_ = nullptr;  // for watching drone's flight state
  boost::thread* thread_publish_setpoint_ = nullptr;  // for publishing drone's setpoint_position/local

  double yaw_degree;
  mavros_msgs::State current_state;
  mavros_msgs::WaypointPush wp_list{};
  mavros_msgs::WaypointList waypoints_;
  mavros_msgs::WaypointReached wp_reached;
  geometry_msgs::PoseStamped local_position;
  geometry_msgs::PoseStamped setpoint;
  ros::Publisher local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

  void getlocalPositionCB(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void getWaypointsFromQGCPlan(const std::string& qgc_plan_file);
  void waypointsCB(const mavros_msgs::WaypointList::ConstPtr &msg);
  void waypointReachedCB(const mavros_msgs::WaypointReached::ConstPtr &msg);
  void publishSetpointThread();
};



AeroDrone::AeroDrone()
{
  resetHome();
  getHomeGeoPoint();

  // Thread that watch for change in Aero flight mode changes.
  thread_watch_flight_mode_ = new boost::thread(boost::bind(&AeroDrone::watchFlightModeThread, this));
}

AeroDrone::~AeroDrone()
{
  delete thread_watch_flight_mode_;
  delete thread_publish_setpoint_;
}

bool AeroDrone::arm()
{
  if (!home_set_)
  {
    ROS_ERROR("Can't arm: No GPS Fix!");
    return false;
  }

  auto arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm;
  srv_arm.request.value = true;
  if (arming_client.call(srv_arm) && srv_arm.response.success)
    return true;
  else
    return false;
}

bool AeroDrone::disarm()
{
  if (!home_set_)
  {
    ROS_ERROR("Can't disarm: No GPS Fix!");
    return false;
  }

  auto disarm_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv_disarm;
  srv_disarm.request.value = false;
  if (disarm_client.call(srv_disarm) && srv_disarm.response.success)
    return true;
  else
    return false;
}

bool AeroDrone::takeoff()
{
  if (!home_set_)
  {
    ROS_ERROR("Can't takeoff: No GPS Fix!");
    return false;
  }

  auto takeoff_client = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff{};

  srv_takeoff.request.altitude = 3; //home_.geo.altitude;
  srv_takeoff.request.latitude = home_.geo.latitude;
  srv_takeoff.request.longitude = home_.geo.longitude;

  if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
    return true;
  else
    return false;
}

bool AeroDrone::land()
{
  if (!home_set_)
  {
    ROS_ERROR("Can't land: No GPS Fix!");
    return false;
  }

  auto land_client = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land{};

  if (land_client.call(srv_land) && srv_land.response.success)
    return true;
  else
    return false;
}

void AeroDrone::resetHome()
{
  home_.geo.latitude = home_.geo.longitude = home_.geo.altitude = NAN;
}

void AeroDrone::getHomeGeoPoint()
{
  // FCU Home position: See http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
  auto home_sub = nh_.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 1,
                                                           boost::bind(&AeroDrone::setHomeGeoPointCB, this, _1));

  ROS_INFO("Waiting for Aero FC Home to be set...");
  while (ros::ok() && !home_set_)
  {
    ros::spinOnce();
    rate_.sleep();
  }

  home_sub.shutdown(); // 获取home点坐标后不再需要继续订阅该值
}

// Callback that gets called periodically from MAVROS notifying Global Poistion of Aero FCU
void AeroDrone::setHomeGeoPointCB(const mavros_msgs::HomePositionConstPtr& home)
{
  home_ = *home;
  home_set_ = true;
  ROS_INFO("Received Home (WGS84 datum): %lf, %lf, %lf", home_.geo.latitude, home_.geo.longitude, home_.geo.altitude);
}

void AeroDrone::getFlightModeCB(const mavros_msgs::StateConstPtr& msg)
{
  current_state = *msg;
}

void AeroDrone::getlocalPositionCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  local_position = *msg;
  double yd;
  if(local_position.pose.orientation.z >= 0)
  yd = 360 * acos(local_position.pose.orientation.w) / M_PI;
  else
  yd = -360 * acos(local_position.pose.orientation.w) / M_PI;

  if(yd >= 0)
  yaw_degree = yd - 360;
  else
  yaw_degree = yd + 360;
}

void AeroDrone::waypointsCB(const mavros_msgs::WaypointList::ConstPtr &msg)
{
  waypoints_ = *msg;
}

void AeroDrone::waypointReachedCB(const mavros_msgs::WaypointReached::ConstPtr &msg)
{
  wp_reached = *msg;
}

void AeroDrone::watchFlightModeThread()
{
  ros::Subscriber state_sub = nh_.subscribe<mavros_msgs::State>(
    "mavros/state", 5, boost::bind(&AeroDrone::getFlightModeCB, this, _1));

  ros::Subscriber local_position_sub = nh_.subscribe<geometry_msgs::PoseStamped>(
    "mavros/local_position/pose", 5, boost::bind(&AeroDrone::getlocalPositionCB, this, _1));

  ros::Subscriber waypoints_sub = nh_.subscribe<mavros_msgs::WaypointList>(
    "/mavros/mission/waypoints", 5, boost::bind(&AeroDrone::waypointsCB, this, _1));

  ros::Subscriber waypoint_reached_sub = nh_.subscribe<mavros_msgs::WaypointReached>(
    "/mavros/mission/reached", 5, boost::bind(&AeroDrone::waypointReachedCB, this, _1));

  while (ros::ok())
  {
    ros::spinOnce();
    rate_.sleep();
  }
}

void AeroDrone::publishSetpointThread()
{
  while (ros::ok()) //current_state.mode == "OFFBOARD"
  {
    local_pos_pub.publish(setpoint);
    ros::spinOnce();
    rate_.sleep();
  }
  thread_publish_setpoint_ = nullptr;
}



mavros_msgs::State AeroDrone::currentState()
{
  return current_state;
}

geometry_msgs::PoseStamped AeroDrone::localPosition()
{
  return local_position;
}

bool AeroDrone::setMode(string modeName)
{
  if (!home_set_)
  {
    ROS_ERROR("Can't set mode: No GPS Fix!");
    return false;
  }


  // send a few setpoints before offboard mode starting and open thread for publishing setpoint
  if(modeName == "OFFBOARD")
  {
    for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(local_position);
    rate_.sleep();
    }
    setpoint = local_position;
  }
  

  ros::ServiceClient setMode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  mavros_msgs::SetMode srv_setMode;
  srv_setMode.request.custom_mode = modeName;

  if (setMode_client.call(srv_setMode) && srv_setMode.response.mode_sent){
    // Thread that publish setpoint.
    thread_publish_setpoint_ = new boost::thread(boost::bind(&AeroDrone::publishSetpointThread, this));

    return true;
  }
  else
    return false;
}


void AeroDrone::pubLocalPos(geometry_msgs::PoseStamped sp)
{
      setpoint = sp;
}

void AeroDrone::getWaypointsFromQGCPlan(const std::string& qgc_plan_file)
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
    for (auto& mi : mission_pt.get_child("mission.items"))
    {
      // See http://docs.ros.org/api/mavros_msgs/html/msg/Waypoint.html
      mavros_msgs::Waypoint wp{};
      // we have mission item now
      wp.frame = mi.second.get<int>("frame");
      wp.command = mi.second.get<int>("command");
      wp.autocontinue = mi.second.get<bool>("autoContinue");
      // Only 1st mission item should be set to true.
      wp.is_current = first ? true : false;
      first = false;
      // Parameters
      std::vector<double> params;
      int param_num = 0;
      for (auto& p : mi.second.get_child("params"))
      {
        try{
          params.push_back(p.second.get<double>(""));
          param_num ++;
        }
        catch (std::exception const& e)
        {
          params.push_back(0);
          ROS_ERROR("Because %s, set param[%d] to zero. ", e.what(), param_num);
        }
      }
      wp.param1 = params[0]; //Hold time at waypoint in decimal seconds (65535 is max)
      wp.param2 = params[1]; //Acceptance radius in meters (when plain inside the sphere of this radius, 
                             //the waypoint is considered reached) (Plane only).
      wp.param3 = params[2]; //0 to pass through the WP, if > 0 radius in meters to pass by WP. 
                             //Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
      wp.param4 = params[3]; //Desired yaw angle at waypoint target.(rotary wing)
      wp.x_lat = params[4]; //Target latitude. If zero, the Copter will hold at the current latitude.
      wp.y_long = params[5]; //Target longitude. If zero, the Copter will hold at the current longitude.
      wp.z_alt = params[6]; //Target altitude. If zero, the Copter will hold at the current altitude.
      // Add it to Waypoint List
      wp_list.request.waypoints.push_back(wp);
    }
    //////////////////////////////////////////////////////////
    // Parse QGC plan ends
  }
  catch (std::exception const& e)
  {
    ROS_ERROR("getWaypointsFromQGCPlan %s", e.what());
    throw;
  }
}


  // Send WPs to Vehicle
void AeroDrone::uploadMission(const std::string& qgc_plan_file)
{
  try
  {
    getWaypointsFromQGCPlan(qgc_plan_file);
  }
  catch (std::exception const& e)
  {
    // NOTE: QGC waypointplan (JSON) may contain 'params' valueas 'null';
    // in that case we may get execption. Make sure to keep 'params' values to be 0.
    ROS_ERROR("Fatal: error in loading waypoints from the file %s!!", qgc_plan_file.c_str());
    abort();
  }

  ros::ServiceClient wp_client = nh_.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
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
        break;
      }
    }
  }
}

mavros_msgs::WaypointList AeroDrone::waypoints()
{
  return waypoints_;
}

int AeroDrone::waypointReached()
{
  return wp_reached.wp_seq;
}


double AeroDrone::yawAngle()
{
  return yaw_degree;
}

void AeroDrone::moveBody(float x, float y, float z)
{ 
  setpoint.pose.position.x = local_position.pose.position.x + cos(yaw_degree * M_PI / 180) * x - sin(yaw_degree * M_PI / 180) * y;
  setpoint.pose.position.y = local_position.pose.position.y + sin(yaw_degree * M_PI / 180) * x + cos(yaw_degree * M_PI / 180) * y;
  setpoint.pose.position.z = local_position.pose.position.z + z;
}

void AeroDrone::moveENU(float x, float y, float z)
{
  setpoint.pose.position.x = local_position.pose.position.x + x;
  setpoint.pose.position.y = local_position.pose.position.y + y;
  setpoint.pose.position.z = local_position.pose.position.z + z;
}

void AeroDrone::moveENUto(float x, float y, float z)
{
  setpoint.pose.position.x = x;
  setpoint.pose.position.y = y;
  setpoint.pose.position.z = z;
}

void AeroDrone::rotateAngle(double degree)
{
  double setyaw = yaw_degree + degree;
  setpoint.pose.orientation.w = cos(setyaw * M_PI / 360);
  setpoint.pose.orientation.z = sin(setyaw * M_PI / 360);
}

void AeroDrone::rotateAngleto(double degree)
{
  double setyaw = degree;
  setpoint.pose.orientation.w = cos(setyaw * M_PI / 360);
  setpoint.pose.orientation.z = sin(setyaw * M_PI / 360);
}