 /********************************************************
 * path_planing.cpp
 * path_planing use octomap method
 * 
 * Author： Born Chow
 * Date: 2020.09.01
 * 
 * 说明:在octomap上实现rrt导航,用于小车的起伏地形导航
 * 这里实现的是流程控制类 planner类 与流程控制ros节点
 * 
 ******************************************************/

#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <octomap/math/Utils.h>

#include <iostream>
#include <vector> 
#include <map>
#include <ctime>
#include "PathFinder.hpp"
#include "data_type.h"

// Declear some global variables




class planner
{
private:
	//ROS相关
	ros::NodeHandle n_;
	ros::Subscriber octree_sub_;
	ros::Subscriber odom_sub_;
	ros::Subscriber goal_sub_;
	//ROS publishers
	ros::Publisher vis_pub_;
	ros::Publisher traj_pub_;
	ros::Publisher rrt_tree_pub_;
	ros::Publisher rrt_path_pub_;

	// ros参数
	string odom_topic_;
	//与前端rrt相关ros参数
	float search_radius_, th_stdev_, setp_eta_;
	int max_iter_;

	vector<octomap::point3d> path_;  

	// Flag for initialization
	bool set_start_ = false;

	shared_ptr<octomap::OcTree> map_tree_;
	float map_resolution_;
	geometry_msgs::Point start_point_, goal_point_, pre_goal_point_;
	Eigen::Vector3d car_body_size_;
	
	// for vis
	visualization_msgs::Marker marker, line, line_path; // 显示采样点, 显示rrt树, 与最终路径

	int nu;
	// UGV trajectory
	trajectory_msgs::MultiDOFJointTrajectory msg;
	trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;

	//shared_ptr<RRTPathFinder> path_finder;
	RRTPathFinder *path_finder = new RRTPathFinder();




private:
	void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg);
	void odomCb(const nav_msgs::Odometry::ConstPtr &msg);
	void startCb(const geometry_msgs::PointStamped::ConstPtr &msg);
	void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg);
	bool point_equal(double x, double y, double z, geometry_msgs::Point point){
		if(abs(point.x - x)< 0.00001 && abs(point.y - y)< 0.00001 && abs(point.z - z)< 0.00001){
			return true;
		}else
		{
			return false;
		}	
	}


public:

	planner(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
	~planner();
	void init_start(void);
	void setGoal(double x, double y, double z);
	void plan(void);
	void replan(void);
	
};

planner::planner(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : n_(nh)
{
	//init start_point
	start_point_.x = 0;
	start_point_.y = 0;
	start_point_.z = 0;

	// load car size
	// carBodySize(0) = 0.6;  //x
	// carBodySize(1) = 0.8;  //y
	// carBodySize(2) = 0.8;  //z  轮底到激光雷达的距离

	init_start(); //测试用

	private_nh.param<string>("odom_topic", odom_topic_, "odom");
	private_nh.param<double>("car_size_x", car_body_size_(0), 0.6);
	private_nh.param<double>("car_size_y", car_body_size_(1), 0.8);
	private_nh.param<double>("car_size_z", car_body_size_(2), 0.8);
	private_nh.param<float>("path_finder/search_radius", search_radius_, 4.0);
	private_nh.param<float>("path_finder/th_stdev", th_stdev_, 1.0);
	private_nh.param<float>("path_finder/setp_eta", setp_eta_, 2.0);
	private_nh.param<int>("path_finder/max_iter", max_iter_, 400000);


	octree_sub_ = n_.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &planner::octomapCallback, this);
 	odom_sub_ = n_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &planner::odomCb, this);
	goal_sub_ = n_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &planner::goalCb, this);

	vis_pub_ = n_.advertise<visualization_msgs::Marker>( "rrt_sample_node", 10 ); // 发布采样点
	traj_pub_ = n_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",10);  // 发布最终轨迹
	rrt_tree_pub_ = n_.advertise<visualization_msgs::Marker>("rrt_tree", 10); //发布rrt 树
    rrt_path_pub_ = n_.advertise<visualization_msgs::Marker>("rrt_path", 10);	//发布回溯rrt节点

	path_finder->initParam(search_radius_, th_stdev_, setp_eta_, max_iter_, car_body_size_);

	// vis setting
	// 红色maker表示查询的点
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "path";
	marker.id = nu;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.15;
	marker.scale.y = 0.15;
	marker.scale.z = 0.15;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	// 白色线段表示rrt　tree
	line.header.frame_id= "map";
    line.header.stamp=ros::Time(0);
    line.ns = "lines";
    line.id = 1;
    line.type=line.LINE_LIST;
    line.action=line.ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x =  0.1;
    line.scale.y = 0.1;
    line.color.r =255.0/255.0;
    line.color.g= 255.0/255.0;
    line.color.b =255.0/255.0;
    line.color.a = 1.0;
    line.lifetime = ros::Duration();
	// 红色线段表示 最终路径
    line_path.header.frame_id= "map";
    line_path.header.stamp=ros::Time(0);
    line_path.ns = "line_back";
    line_path.id = 2;
    line_path.type=line.LINE_LIST;
    line_path.action=line.ADD;
    line_path.pose.orientation.w = 1.0;
    line_path.scale.x = 0.12;
    line_path.scale.y= 0.12;
    line_path.color.r =255.0/255.0;
    line_path.color.g= 0.0/255.0;
    line_path.color.b =0.0/255.0;
    line_path.color.a = 1.0;
    line_path.lifetime = ros::Duration();

 
	std::cout<<"[planner] Initialized"<<std::endl;

}

planner::~planner()
{
}

void planner::init_start(void)
{
	if(!set_start_)
		std::cout << "[planner] Start point set " << std::endl;
	set_start_ = true;
}

void planner::setGoal(double x, double y, double z){
	if (!point_equal(x,y,z, pre_goal_point_))
	{
		goal_point_.x = x;
		goal_point_.y = y;
		goal_point_.z = z;
		pre_goal_point_ = goal_point_;

		// vis 显示目标点 测试用
		marker.ns = "goal";
		marker.id = 1;
		marker.scale.x = 0.3;
		marker.scale.y = 0.3;
		marker.scale.z = 0.3;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = z;
		vis_pub_.publish(marker);
		// vis

		cout << "[planner] Goal point set to: " << x << " " << y << " " << z << std::endl;
		if(set_start_){
			plan();
		}else
		{
			cout<<"[planner] no Start point set !!"<< endl;
		}
		
	}
	
}


void planner::plan(void){
	cout<<"start plan"<<endl;
	octomap::point3d base_start(start_point_.x, start_point_.y, start_point_.z);
	octomap::point3d base_goal(goal_point_.x, goal_point_.y, goal_point_.z);

	TicToc time_rrt;
	path_finder->updateMap(map_tree_);
	path_finder->pathSearch(base_start, base_goal);
	time_rrt.toc("time rrt finder： ");

	line_path.points.clear();
	line.points.clear();
	geometry_msgs::Point point_pub;
	
}

void planner::replan(void){
	// plan();
}


void planner::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
	// convert octree to collision object
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	map_tree_ = shared_ptr<octomap::OcTree>(tree_oct);
	map_resolution_ = map_tree_->getResolution();
	cout<<"map Resolution: "<< map_resolution_ <<endl;
	replan();
}

void planner::odomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
	start_point_.x = msg->pose.pose.position.x;
	start_point_.y = msg->pose.pose.position.y;
	start_point_.z = msg->pose.pose.position.z;
	init_start();
}

void planner::startCb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	start_point_.x = msg->point.x;
	start_point_.y = msg->point.y;
	start_point_.z = msg->point.z;
	init_start();
}

void planner::goalCb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	setGoal(msg->point.x, msg->point.y, msg->point.z);
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>class end<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc, char **argv)
{
	ros::init(argc, argv, "octomap_planner");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh;
	//planner planner_object;

	planner planner_object(nh, private_nh);

	ros::spin();

	return 0;
}
