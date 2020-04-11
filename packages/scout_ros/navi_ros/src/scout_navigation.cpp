#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_costmap");
    ros::NodeHandle nh;

    tf::TransformListener tf_(ros::Duration(10));

    costmap_2d::Costmap2DROS *global_costmap_ros_ =  new costmap_2d::Costmap2DROS("global_costmap", tf_);
    global_costmap_ros_->pause();

    costmap_2d::Costmap2DROS *local_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    local_costmap_ros_->pause();

    global_costmap_ros_->start();
    local_costmap_ros_->start();
    ros::spin();
    return 0;

}