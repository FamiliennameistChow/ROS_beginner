#include <ros/ros.h>
#include "MergeOccupyMap.hpp"

int main(int argc, char ** argv){
    ros::init(argc, argv, "merge_occupy_map");

    ros::NodeHandle nh("~");
    ros::Rate rate(30.0);

    gridmap_scout::MergeOccupyMap mergeOccupyMap(nh);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}