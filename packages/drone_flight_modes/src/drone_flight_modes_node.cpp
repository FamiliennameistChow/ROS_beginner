/**
*@file aero_flight_modes_node.cpp
*@brief Demonstration of getting Flight modes using mavros
*@date 2017-10-24
*/

#include <cstdlib>
#include "drone_flight_modes.hpp"
#include <time.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aero_flight_modes");
  ros::NodeHandle nh;

  ros::Rate rate(20.0);
  AeroDrone aero;
  int ret = EXIT_SUCCESS;

  geometry_msgs::PoseStamped setpoint;
  setpoint.pose.position.x = 0;
  setpoint.pose.position.y = 0;
  setpoint.pose.position.z = 5;


  // Arm
  if (!aero.arm())
  {
    ROS_ERROR("Fatal: Arming failed!");
    ret = EXIT_FAILURE;
    goto end;
  }
  else
  {
    ROS_INFO("Arm sent");
  }

  // Takeoff
  if (!aero.takeoff())
  {
    ROS_ERROR("Fatal: Takeoff failed!");
    ret = EXIT_FAILURE;
    goto end;
  }
  else
  {
    ROS_INFO("Takeoff sent");
  }

  sleep(5);
  ROS_INFO("Battery remaining: %g%%",aero.batteryState().percentage * 100.f);

  // set to offboard mode 
  if (!aero.setMode("OFFBOARD"))
  {
    ROS_ERROR("Fatal: Set OFFBOARD mode failed!");
    ret = EXIT_FAILURE;
    goto end;
  }
  else
  {
    ROS_INFO("Set OFFBOARD mode sent");
  }

  // publish a setpoint 
  aero.pubLocalPos(setpoint);

  sleep(10);
  ROS_INFO("Battery remaining: %g%%",aero.batteryState().percentage * 100.f);

  // move based on body coordinate
  aero.moveBody(5, 5, -2);

  sleep(5);
  ROS_INFO("Battery remaining: %g%%",aero.batteryState().percentage * 100.f);

  // Fly a circle
  ROS_INFO("Body: Fly a circle");
  aero.setVelocityBody(5.0f, 0.0f, 0.0f, -30.0f);
  
  sleep(6);
  ROS_INFO("body: Wait for a bit");
  aero.setVelocityBody(0.0f, 0.0f, 0.0f, 0.0f);
  
  sleep(10);
  ROS_INFO("Battery remaining: %g%%",aero.batteryState().percentage * 100.f);

  // move based on body coordinate
  aero.moveBody(0, 0, 2);

  sleep(5);
  ROS_INFO("Battery remaining: %g%%",aero.batteryState().percentage * 100.f);

  // fly mission plan
  aero.uploadMission("/home/danny/fly_mission.plan");
  if (!aero.setMode("AUTO.MISSION"))
  {
    ROS_ERROR("Fatal: Set AUTO.MISSION mode failed!");
    ret = EXIT_FAILURE;
    goto end;
  }
  else
  {
    ROS_INFO("Set AUTO.MISSION mode sent");
  }
  while (ros::ok()) {
    // ROS_INFO("Mode is %s",aero.currentState().mode.c_str());
    // ROS_INFO("waypoint reached %d",aero.waypointReached());
    // ROS_INFO("current is %d",aero.waypoints().current_seq);
    // ROS_INFO("yawAngle is %f",aero.yawAngle());
    if(aero.waypointReached() == (aero.waypoints().waypoints.size()-2)){
      ROS_INFO("mission complete");
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }


  if (!aero.setMode("OFFBOARD"))
  {
    ROS_ERROR("Fatal: Set OFFBOARD mode failed!");
    ret = EXIT_FAILURE;
    goto end;
  }
  else
  {
    ROS_INFO("Set OFFBOARD mode sent");
  }

  aero.pubLocalPos(setpoint);

  sleep(15);

  // Land
  if (!aero.land())
  {
    ROS_ERROR("Fatal: Land failed!");
    ret = EXIT_FAILURE;
    goto end;
  }
  else
  {
    ROS_INFO("Land sent");
  }

  sleep(15);  // Let Aero land..
  ROS_INFO("Battery remaining: %g%%",aero.batteryState().percentage * 100.f);

  // disarm
  if (!aero.disarm())
  {
    ROS_ERROR("Fatal: disarm failed!");
    ret = EXIT_FAILURE;
    goto end;
  }
  else
  {
    ROS_INFO("disarm sent");
  }

  sleep(5);
  ROS_INFO("Battery remaining: %g%%",aero.batteryState().percentage * 100.f);

  ROS_INFO("Done");

end:
  ros::shutdown();

  return ret;
}
