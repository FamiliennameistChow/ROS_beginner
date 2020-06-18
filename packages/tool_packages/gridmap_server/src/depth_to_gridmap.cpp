#include <ros/ros.h>
#include "DepthToGridmap.hpp"
#include <time.h>

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "depth_to_gridmap");
  ros::NodeHandle nh("~");
  ros::Rate rate(30);
  gridmap_server::DepthToGridmap depthToGridmap(nh);

  while(ros::ok())
  {
      ros::spinOnce();
      rate.sleep();
  }
  return 0;
}