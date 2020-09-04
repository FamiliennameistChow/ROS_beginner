 /********************************************************
 * path_planing.cpp
 * path_planing use octomap method
 * 
 * Author： Born Chow
 * Date: 2020.09.01
 * 
 * 说明:在octomap上实现rrt导航,用于小车的起伏地形导航
 * 这里实现的是流程控制类 planner类 流程控制ros节点
 * 
 ******************************************************/

#include "ros/ros.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <octomap/math/Utils.h>

#include <iostream>
#include <vector> 
#include <map>
#include <ctime>
#include <thread>
#include <mutex>
#include <math.h>


#include "data_type.h"
#include "tictoc.h"
#include "PathFinder.hpp"

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
	ros::Publisher traj_pub_;
	ros::Publisher rrt_tree_pub_;
	ros::Publisher rrt_path_pub_;
	ros::Publisher vis_pub_, vis_goal_pub_, vis_corridor_pub_;


	// >>>>>>>>>>ros参数
	string odom_topic_;
	//与前端rrt相关ros参数
	float search_radius_, th_stdev_, setp_eta_;
	int max_iter_;

	//车体参数
	Eigen::Vector3d car_body_size_;
	
	// corridor相关
	int max_inflate_iter_; // 最大膨胀次数
	bool is_proj_cube_; // 是否扁平显示
	float inflate_step_length_; //每次膨胀步长
	// <<<<<<<<<<ros参数
	
	std::mutex mtx;

	vector<octomap::point3d> path_;  //前端rrt路径点

	// 流程相关
	bool set_start_ = false;
	shared_ptr<octomap::OcTree> map_tree_;
	double map_resolution_;
	geometry_msgs::Point start_point_, goal_point_, pre_goal_point_;
	
	// >>>>>>>>>>>>>>>vis for rrt
	visualization_msgs::MarkerArray node_vis_;
	visualization_msgs::Marker marker, line, line_path; // 显示采样点, 显示rrt树, 与最终路径
	vector<octomap::point3d> rrt_path_; //rrt最终路径节点集合
	vector<octomap::point3d> pub_new_pt_set_; //rrt采样点集合
	vector<octomap::point3d> pub_block_pt_set_; //rrt阻断的节点集合
	vector<pair<octomap::point3d, octomap::point3d> > pub_rrt_tree_set_; // rrt树
	int nu;
	geometry_msgs::Point point_pub_;
	// <<<<<<<<<<<<<<<<vis for rrt

	// >>>>>>>>>>>>>>>>>>>vis for corridor
	visualization_msgs::MarkerArray cube_vis_;
	// <<<<<<<<<<<<<<<<<<<vis for corridor

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
	// rrt 相关显示
	void visRRTFlow();
	void visRRTPath(vector<octomap::point3d> path);
	// corridor相关
	std::vector<Cube> corridorGeneration(vector<octomap::point3d> path);
	Cube generateCube(octomap::point3d pt);
	pair<Cube, bool> inflateCube(Cube cube, Cube lstcube);
	bool isContains(Cube cube1, Cube cube2)
	{   
		// if( cube1.vertex(0, 0) >= cube2.vertex(0, 0) && cube1.vertex(0, 1) <= cube2.vertex(0, 1) && cube1.vertex(0, 2) >= cube2.vertex(0, 2) &&
		// 	cube1.vertex(6, 0) <= cube2.vertex(6, 0) && cube1.vertex(6, 1) >= cube2.vertex(6, 1) && cube1.vertex(6, 2) <= cube2.vertex(6, 2)  )
		if( cube1.vertex(0, 0) >= cube2.vertex(0, 0) && cube1.vertex(0, 1) <= cube2.vertex(0, 1) &&
			cube1.vertex(6, 0) <= cube2.vertex(6, 0) && cube1.vertex(6, 1) >= cube2.vertex(6, 1)  )
			return true;
		else
			return false; 
	}

	
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

	void visualizeRRTThread();
	void visCorridor(vector<Cube> corridor);
	
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
	private_nh.param<float>("path_finder/th_stdev", th_stdev_, 0.1);
	private_nh.param<float>("path_finder/setp_eta", setp_eta_, 2.0);
	private_nh.param<int>  ("path_finder/max_iter", max_iter_, 10000);
	private_nh.param<bool> ("vis/is_proj_cube", is_proj_cube_, false);
	private_nh.param<int>  ("planner/max_inflate", max_inflate_iter_, 10);
	private_nh.param<float>("planner/inflate_step_length", inflate_step_length_, 1.0);


	octree_sub_ = n_.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &planner::octomapCallback, this);
 	odom_sub_ = n_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &planner::odomCb, this);
	goal_sub_ = n_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &planner::goalCb, this);

	
	traj_pub_ = n_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",10);  // 发布最终轨迹
	vis_pub_ = n_.advertise<visualization_msgs::MarkerArray>( "rrt_sample_node", 10 ); // 发布采样点
	vis_goal_pub_ = n_.advertise<visualization_msgs::Marker>( "goal_node", 10 ); // 发布采样点
	rrt_tree_pub_ = n_.advertise<visualization_msgs::Marker>("rrt_tree", 10); //发布rrt 树
    rrt_path_pub_ = n_.advertise<visualization_msgs::Marker>("rrt_path", 10);	//发布回溯rrt节点
	vis_corridor_pub_  = nh.advertise<visualization_msgs::MarkerArray>("corridor_vis", 1);


	path_finder->initParam(search_radius_, th_stdev_, setp_eta_, max_iter_, car_body_size_, n_, true);

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
    line.ns = "rrt_tree";
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
    line_path.ns = "rrt_path";
    line_path.id = 1;
    line_path.type=line.LINE_STRIP;
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
		vis_goal_pub_.publish(marker);
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

	// 步骤一: rrt 前端　路径搜索
	octomap::point3d base_start(start_point_.x, start_point_.y, start_point_.z);
	octomap::point3d base_goal(goal_point_.x, goal_point_.y, goal_point_.z);

	// >>>>>>>>清空显示数据
	nu = 2;
	// TODO 清除 rrt_tree 与　rrt_path在rviz上的显示　？？？
	line.points.clear();
	line_path.points.clear();
	rrt_path_pub_.publish(line_path);
	rrt_tree_pub_.publish(line);

	line.id = line_path.id = rand() % 10000; // id需要改变，不然line_path.points.clear()之后，rviz会崩溃

	for (auto & mk : node_vis_.markers)
	{
		mk.action = visualization_msgs::Marker::DELETE;
	}
	vis_pub_.publish(node_vis_);
	node_vis_.markers.clear();
	// <<<<<<<<<清空显示数据

	rrt_path_.clear();

	TicToc time_rrt;
	path_finder->updateMap(map_tree_);
	path_finder->pathSearch(base_start, base_goal);
	if(path_finder->getPath(rrt_path_)){
		visRRTPath(rrt_path_);
	}
	time_rrt.toc("time rrt finder: ");
	cout << "[rrt state] : " << path_finder->getSearchState() << endl;

	// 步骤二: corridor生成　需要 rrt_path_ , map_tree_
	vector<Cube> corridor;

	TicToc time_corridor_gen;
	corridor = corridorGeneration(rrt_path_);
	time_corridor_gen.toc("time time corridor gen: ");
	cout << "corridor size: " << corridor.size()<< endl;

	visCorridor(corridor);
	
}

void planner::replan(void){
	// plan();
}

std::vector<Cube> planner::corridorGeneration(vector<octomap::point3d> path){
	vector<Cube> cubeList;
	octomap::point3d pt;
	Cube lastCube;

	for (size_t i = 0; i < (int)path.size(); i++)
	{
		
		pt = path[i];
		cout << "path pt: " << pt(0) << " " << pt(1) << " " << pt(2) << endl;
		Cube cube = generateCube(pt);
		auto result = inflateCube(cube, lastCube);

		if(result.second == false)
            continue;

        cube = result.first;

		lastCube = cube;
		cubeList.push_back(cube);
	}
	return cubeList;
	
}


Cube planner::generateCube(octomap::point3d pt){
/**********************************************************
           P3------------P2 
           /|           /|              ^
          / |          / |              | z
        P0--|---------P1 |              |
         |  P7--------|--p6             |
         | /          | /               /--------> y
         |/           |/               /  
        P4------------P5              / x

	以路径点为中心生成初始立方体, 点的膨胀尺度为一个地图分辨率
	实现参考：https://github.com/HKUST-Aerial-Robotics/Btraj
***********************************************************/ 
	Cube cube;
	
	cube.center(0) = pt(0);
	cube.center(1) = pt(1);
	cube.center(2) = pt(2);

	double x_u = pt(0) + map_resolution_;
    double x_l = pt(0) - map_resolution_;
    
    double y_u = pt(1) + map_resolution_;
    double y_l = pt(1) - map_resolution_;
    
    double z_u = pt(2) + map_resolution_;
    double z_l = pt(2) - map_resolution_;

	cube.vertex.row(0) = Eigen::Vector3d(x_u, y_l, z_u);  
    cube.vertex.row(1) = Eigen::Vector3d(x_u, y_u, z_u);  
    cube.vertex.row(2) = Eigen::Vector3d(x_l, y_u, z_u);  
    cube.vertex.row(3) = Eigen::Vector3d(x_l, y_l, z_u);  

    cube.vertex.row(4) = Eigen::Vector3d(x_u, y_l, z_l);  
    cube.vertex.row(5) = Eigen::Vector3d(x_u, y_u, z_l);  
    cube.vertex.row(6) = Eigen::Vector3d(x_l, y_u, z_l);  
    cube.vertex.row(7) = Eigen::Vector3d(x_l, y_l, z_l);  

    return cube;

}


pair<Cube, bool> planner::inflateCube(Cube cube, Cube lstcube){
/******************************************
 * 基于初始立方体的通行走廊生成
 * 具体方法为向八个方向膨胀
 * 
 * 
******************************************/
	Cube cubeMax = cube;
	Eigen::MatrixXd vertex_idx(8, 3);

	vertex_idx = cube.vertex;

	double id_x, id_y, id_z;

	/*
	       P3------------P2 
           /|           /|              ^
          / |          / |              | z
        P0--|---------P1 |              |
         |  P7--------|--p6             |
         | /          | /               /--------> y
         |/           |/               /  
        P4------------P5              / x

	*/

	int iter = 0;
	bool collide;

	while (iter < max_inflate_iter_)
	{
		// 向八个方向膨胀
		// Y- 将初始立方体向Y-方向膨胀　(膨胀面: p0 -- p3 -- p7 -- p4 )
		//***********************************************************************************
		collide = false;
		int y_lo = vertex_idx(0, 1) - inflate_step_length_;

		cout << "-------------------------------" <<iter << endl;
		cout << "Y-" << endl;
		for (id_y = vertex_idx(0, 1); id_y >= y_lo; id_y = id_y - 3*map_resolution_)
		{
			cout << "id_y: " << id_y << endl;
			if (collide) break;

			for (id_x = vertex_idx(0, 0); id_x >= vertex_idx(3, 0); id_x = id_x - 3*map_resolution_)
			{
				//cout << "vertex_idx(0, 0): " << vertex_idx(0, 0) << " vertex_idx(3, 0)" << vertex_idx(3, 0) << " id_x " << id_x <<endl;
				if (collide) break;

				id_z = vertex_idx(0, 2); 

				octomap::point3d p(id_x, id_y, id_z);
				if (!path_finder->validGround(p, th_stdev_+0.1))
				{
					collide = true;
					break;
				}
			}	
		}

		if (collide)
		{
			//与边界保持3个分辨率的距离
			vertex_idx(0, 1) = min(id_y+6*map_resolution_, vertex_idx(0, 1));
            vertex_idx(3, 1) = min(id_y+6*map_resolution_, vertex_idx(3, 1));
            vertex_idx(7, 1) = min(id_y+6*map_resolution_, vertex_idx(7, 1));
            vertex_idx(4, 1) = min(id_y+6*map_resolution_, vertex_idx(4, 1));
		}else
		{
			vertex_idx(0, 1) = vertex_idx(3, 1) = vertex_idx(7, 1) = vertex_idx(4, 1) = id_y + map_resolution_;
		}

		// Y+ 将初始立方体向Y+方向膨胀　(膨胀面: p1 -- p2 -- p6 -- p5 )
		//***********************************************************************************
		collide = false;
		int y_up = vertex_idx(1, 1) + inflate_step_length_;

		cout << "Y+" << endl;
		for (id_y = vertex_idx(1, 1); id_y <= y_up; id_y = id_y + 3*map_resolution_)
		{
			if (collide) break;

			for (id_x = vertex_idx(1, 0); id_x >= vertex_idx(2, 0); id_x = id_x - 3*map_resolution_)
			{
				if (collide) break;

				id_z = vertex_idx(1, 2);

				octomap::point3d p(id_x, id_y, id_z);
				if (!path_finder->validGround(p, th_stdev_+0.1))
				{
					collide = true;
					break;
				}
			}
		}

		if (collide)
		{
			//与边界保持3个分辨率的距离
			vertex_idx(1, 1) = max(id_y-6*map_resolution_, vertex_idx(1, 1));
            vertex_idx(2, 1) = max(id_y-6*map_resolution_, vertex_idx(2, 1));
            vertex_idx(5, 1) = max(id_y-6*map_resolution_, vertex_idx(5, 1));
            vertex_idx(6, 1) = max(id_y-6*map_resolution_, vertex_idx(6, 1));
		}else
		{
			vertex_idx(1, 1) = vertex_idx(2, 1) = vertex_idx(6, 1) = vertex_idx(5, 1) = id_y - map_resolution_;
		}

		// X+ 将初始立方体向X+方向膨胀　(膨胀面: p0 -- p1 -- p5 -- p4 )
		//***********************************************************************************
		collide = false;
		int x_up = vertex_idx(0, 0) + inflate_step_length_;

		cout << "X+" << endl;
		for (id_x = vertex_idx(0, 0); id_x <= x_up; id_x = id_x + 3*map_resolution_)
		{
			if (collide) break;

			for (id_y = vertex_idx(0, 1); id_y <= vertex_idx(1, 1); id_y = id_y + 3*map_resolution_)
			{
				if (collide) break;

				id_z = vertex_idx(0, 2);

				octomap::point3d p(id_x, id_y, id_z);
				if (!path_finder->validGround(p, th_stdev_+0.1))
				{
					collide = true;
					break;
				}
			}
		}

		if(collide)
        {
            vertex_idx(0, 0) = max(id_x-6*map_resolution_, vertex_idx(0, 0)); 
            vertex_idx(1, 0) = max(id_x-6*map_resolution_, vertex_idx(1, 0)); 
            vertex_idx(5, 0) = max(id_x-6*map_resolution_, vertex_idx(5, 0)); 
            vertex_idx(4, 0) = max(id_x-6*map_resolution_, vertex_idx(4, 0)); 
        }
        else{
			vertex_idx(0, 0) = vertex_idx(1, 0) = vertex_idx(5, 0) = vertex_idx(4, 0) = id_x - map_resolution_; 
		}

		// X- 将初始立方体向X-方向膨胀　(膨胀面: p3 -- p2 -- p6 -- p7 )
		//***********************************************************************************
		collide = false;
		int x_lo = vertex_idx(3, 0) - inflate_step_length_;

		cout << "X-" << endl;
		for (id_x = vertex_idx(3, 0); id_x >= x_lo; id_x = id_x - 3*map_resolution_)
		{
			if (collide) break;

			for (id_y = vertex_idx(3, 1); id_y <= vertex_idx(2, 1); id_y = id_y + 3*map_resolution_)
			{
				if (collide) break;

				id_z = vertex_idx(3, 2);

				octomap::point3d p(id_x, id_y, id_z);
				if (!path_finder->validGround(p, th_stdev_+0.1))
				{
					collide = true;
					break;
				}

			}
			
		}

		if(collide)
        {
            vertex_idx(3, 0) = min(id_x+6*map_resolution_, vertex_idx(3, 0)); 
            vertex_idx(2, 0) = min(id_x+6*map_resolution_, vertex_idx(2, 0)); 
            vertex_idx(6, 0) = min(id_x+6*map_resolution_, vertex_idx(6, 0)); 
            vertex_idx(7, 0) = min(id_x+6*map_resolution_, vertex_idx(7, 0)); 
        }
        else{
			vertex_idx(3, 0) = vertex_idx(2, 0) = vertex_idx(6, 0) = vertex_idx(7, 0) = id_x + map_resolution_;
		}
            
		
		// 一次膨胀完成
		//************************************************************************************
		
		cubeMax.setVertex(vertex_idx, map_resolution_);
		if( isContains(lstcube, cubeMax))        
			return make_pair(lstcube, false);

		iter ++;

	}

	return make_pair(cubeMax, true);
}


void planner::visualizeRRTThread(){


	ros::Rate rate(10);
	while (ros::ok())
	{
		if (path_finder->getSearchState())
		{
			continue;
		}
		rate.sleep();
		visRRTFlow();

		//if (path_finder->getSearchState())	break;
	}
	
}

void planner::visRRTPath(vector<octomap::point3d> path){

	octomap::point3d p_pub;
	geometry_msgs::Point point_pub;
	for (size_t i = 0; i < path.size(); i++)
	{
		p_pub = path[i];
		cout<<"[rrt plan]: path size: " << i  <<"--> " << path.size() << endl;
		cout<<"rrt path point: " << p_pub <<endl;
		// trajectory pub
		point_msg.time_from_start.fromSec(ros::Time::now().toSec());
		point_msg.transforms.resize(1);
		point_msg.transforms[0].translation.x = p_pub(0);
		point_msg.transforms[0].translation.y = p_pub(1);
		point_msg.transforms[0].translation.z = p_pub(2);

		point_msg.transforms[0].rotation.x = 0;
		point_msg.transforms[0].rotation.y = 0;
		point_msg.transforms[0].rotation.z = 0;
		point_msg.transforms[0].rotation.w = 1;
		msg.points.push_back(point_msg);
		// trajectory pub 

		//vis
		// cout<<"rrt path first point: " << p_pub <<endl;
		// point_pub.x = p_pub(0);
		// point_pub.y = p_pub(1);
		// point_pub.z = p_pub(2);
		// line_path.points.push_back(point_pub);
		// p_pub= path[i+1];
		// cout<<"rrt path second point: " << p_pub <<endl;
		// point_pub.x = p_pub(0);
		// point_pub.y = p_pub(1);
		// point_pub.z = p_pub(2);
		// line_path.points.push_back(point_pub);
		// rrt_path_pub_.publish(line_path);

		point_pub.x = p_pub(0);
		point_pub.y = p_pub(1);
		point_pub.z = p_pub(2);
		line_path.points.push_back(point_pub);
		rrt_path_pub_.publish(line_path);
		//vis
	}

	traj_pub_.publish(msg);
	
}


void planner::visCorridor(vector<Cube> corridor)
{   
    // >>>>>清除历史数据
    for(auto & mk: cube_vis_.markers) 
        mk.action = visualization_msgs::Marker::DELETE;
    
    vis_corridor_pub_.publish(cube_vis_);

    cube_vis_.markers.clear();
    // <<<<<清除历史数据

    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "corridor";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.a = 0.4;
    mk.color.r = 1.0;
    mk.color.g = 1.0;
    mk.color.b = 1.0;

    int idx = 0;
    for(int i = 0; i < int(corridor.size()); i++)
    {   
        mk.id = idx;

        mk.pose.position.x = (corridor[i].vertex(0, 0) + corridor[i].vertex(3, 0) ) / 2.0; 
        mk.pose.position.y = (corridor[i].vertex(0, 1) + corridor[i].vertex(1, 1) ) / 2.0; 

        if(is_proj_cube_)
            mk.pose.position.z = 0.0;  
        else
            mk.pose.position.z = (corridor[i].vertex(0, 2) + corridor[i].vertex(4, 2) ) / 2.0; 

        mk.scale.x = (corridor[i].vertex(0, 0) - corridor[i].vertex(3, 0) );
        mk.scale.y = (corridor[i].vertex(1, 1) - corridor[i].vertex(0, 1) );

        if(is_proj_cube_)
            mk.scale.z = 0.05; 
        else
            mk.scale.z = (corridor[i].vertex(0, 2) - corridor[i].vertex(4, 2) );

        idx ++;
        cube_vis_.markers.push_back(mk);
    }

    vis_corridor_pub_.publish(cube_vis_);
}

void planner::visRRTFlow(){
	pub_new_pt_set_.clear(); //rrt采样点集合
	pub_block_pt_set_.clear(); //rrt阻断的节点集合
	pub_rrt_tree_set_.clear(); // rrt树
	
	mtx.lock();
	path_finder->getVisNode(pub_new_pt_set_, pub_block_pt_set_, pub_rrt_tree_set_);
	mtx.unlock();

	if (!pub_new_pt_set_.empty())
	{
		for (size_t i = 0; i < pub_new_pt_set_.size(); i++)
		{
			nu++;
			// vis 显示采样点
			marker.id = nu;
			marker.ns = "node";
			marker.scale.x = 0.15;
			marker.scale.y = 0.15;
			marker.scale.z = 0.15;
			marker.color.a = 1.0;
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
			marker.pose.position.x = pub_new_pt_set_[i](0);
			marker.pose.position.y = pub_new_pt_set_[i](1);
			marker.pose.position.z = pub_new_pt_set_[i](2);
			// vis_pub_.publish(marker);
			node_vis_.markers.push_back(marker);
			// vis
		}
		vis_pub_.publish(node_vis_);
	}

	if (!pub_block_pt_set_.empty())
	{
		for (size_t i = 0; i < pub_block_pt_set_.size(); i++)
		{
			nu++;
			// vis 显示阻断的节点 测试用
			marker.ns = "node";
			marker.id = nu;
			marker.scale.x = 0.15;
			marker.scale.y = 0.15;
			marker.scale.z = 0.15;
			marker.color.a = 1.0;
			marker.color.r = 255.0/255.0;
			marker.color.g = 0.0/255.0;
			marker.color.b = 255.0  /255.0;
			marker.pose.position.x = pub_block_pt_set_[i](0);
			marker.pose.position.y = pub_block_pt_set_[i](1);
			marker.pose.position.z = pub_block_pt_set_[i](2);
			// vis_pub_.publish(marker);
			node_vis_.markers.push_back(marker);
			// vis 显示阻断的节点
		}
		vis_pub_.publish(node_vis_);
	}

	if (!pub_rrt_tree_set_.empty())
	{
		for (size_t i = 0; i < pub_rrt_tree_set_.size(); i++)
		{
			// vis 线段表示 rrt tree
			point_pub_.x = pub_rrt_tree_set_[i].first(0);
			point_pub_.y = pub_rrt_tree_set_[i].first(1);
			point_pub_.z = pub_rrt_tree_set_[i].first(2);
			line.points.push_back(point_pub_);
			point_pub_.x = pub_rrt_tree_set_[i].second(0);
			point_pub_.y = pub_rrt_tree_set_[i].second(1);
			point_pub_.z = pub_rrt_tree_set_[i].second(2);
			line.points.push_back(point_pub_);
			rrt_tree_pub_.publish(line);
			// vis
		}
	}
	
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
	std::thread visthread(&planner::visualizeRRTThread, &planner_object);

	ros::spin();

	//visthread.join();

	return 0;
}
