/******************************
 * 
 *  Robot的结构TF可以借助 robot_state_publisher进行发布：在urdf文件中对关键link进行几何配置
 *  这个cpp文件在gazebo中仿真 将 base_link 到 map的 TF
 * 
 ******************************/

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class Robot_TF{
public:
	Robot_TF(ros::NodeHandle& nh);
	~Robot_TF(){}

	void run();

private:
	ros::NodeHandle nh_;
	ros::Rate loop_rate_;
	ros::Subscriber ground_truth_sub_;
	
	tf::TransformBroadcaster broadcaster_;
	tf::Transform tf_base_link_to_map_;

private:
	void ground_truth_sub_cb(const nav_msgs::Odometry::ConstPtr& msg);
};

Robot_TF::Robot_TF(ros::NodeHandle& nh):nh_(nh), loop_rate_(ros::Rate(50))
{
	ground_truth_sub_ = nh_.subscribe<nav_msgs::Odometry>("/ground_truth/odom", 2, &Robot_TF::ground_truth_sub_cb, this);

	tf_base_link_to_map_.setOrigin(tf::Vector3(0.0, 0.0, 2.3));
	tf_base_link_to_map_.setRotation(tf::Quaternion(0, 0, 0, 1));
}

void Robot_TF::ground_truth_sub_cb(const nav_msgs::Odometry::ConstPtr& msg){
	tf_base_link_to_map_.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
	tf_base_link_to_map_.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
											   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
}

void Robot_TF::run(){
	while(nh_.ok()){
		ros::spinOnce();
		broadcaster_.sendTransform(tf::StampedTransform(tf_base_link_to_map_, 
								ros::Time::now(),"map", "base_link"));
		loop_rate_.sleep();
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle nh;
	Robot_TF robot_tf_br = Robot_TF(nh);
	robot_tf_br.run();
}
