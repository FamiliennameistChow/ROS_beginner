#include <ros/ros.h>
#include "PointCloudToGridmap.hpp"
#include <time.h>

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "pointcloud_to_gridmap");
  ros::NodeHandle nh("~");
  ros::Rate rate(10);
  grid_map_uav::PointCloudToGridmap pointCloudToGridmap(nh);

  while(ros::ok())
  {
      ros::spinOnce();
      rate.sleep();
  }
  return 0;
}