/*****************************************************
 * traj_gen.cpp
 * 
 * Author:      Born Chow
 * Date:        2020.08.29
 * Description:
 * 八叉树3D导航, 后端轨迹优化;
 * 采用贝塞儿曲线 + Corridor的方式
 * 
 * 【订阅】八叉树地图
 * 
 * ***************************************************/
 
#include "ros/ros.h"
#include <thread>
#include "mutex"
#include <Eigen/Dense>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>

#include "data_type.h"

using namespace std;

// >>>>>>>>>>>>>>>>>>>>>>>>>> class TrajGen <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

class TrajGen
{
private:
	ros::NodeHandle n_;

	ros::Subscriber octree_sub_;
	
	ros::Publisher traj_pub_;
	ros::Publisher traj_vis_pub_;
	ros::Publisher corridor_vis_pub_;

	shared_ptr<octomap::OcTree> octo_map_;
	float map_resolution_;

private:
	void init_param(ros::NodeHandle &private_nh);	
	void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg);
	
public:
	TrajGen(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
	~TrajGen();

	bool validGround(octomap::point3d query_point, float& mean_);
	vector<Cube> corridorGeneration(vector<Eigen::Vector3d> path_coord);
	
};

TrajGen::TrajGen(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : (n_)nh
{
	init_param(private_nh);

	octree_sub_ = n_.subscribe<octomap_msgs::Octomap>("/octomap_binary", 5, &TrajGen::octomapCallback, this);

	traj_vis_pub_ = n_.advertise<visualization_msgs::Marker>( "visualization_marker_jerk", 10);
	traj_pub_ = n_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/bebop2/command/trajectory",10);
	corridor_vis_pub_ = n_.advertise<visualization_msgs::MarkerArray>("corridor_vis", 1);
}


TrajGen::~TrajGen()
{
}

void TrajGen::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg){
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	octo_map_ = shared_ptr<octomap::OcTree>(tree_oct);
	map_resolution_ = octo_map_->getResolution();

}

vector<Cube> TrajGen::corridorGeneration(vector<Eigen::Vector3d> path_coord);


bool TrajGen::validGround(octomap::point3d query_point, float& mean_){
	// 返回地形高度
	// sreach the ground state
	// sreach pattern
	// 
	//   *****
	//   **#**
	//   *****
	//
	     
	vector <octomap::point3d> sreach_area;
	sreach_area.push_back(query_point);
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution_, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution_, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution_, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution_, query_point(1)                   , query_point(2)));
	// sreach_area.push_back(octomap::point3d(query_point(0) + 3*map_resolution_, query_point(1)                   , query_point(2)));
	// sreach_area.push_back(octomap::point3d(query_point(0) - 3*map_resolution_, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) + 1*map_resolution_, query_point(2)));
	// sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) + 2*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) - 1*map_resolution_, query_point(2)));
	// sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) - 2*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution_, query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution_, query_point(1) - 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution_, query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution_, query_point(1) - 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution_, query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution_, query_point(1) - 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 2*map_resolution_, query_point(1) + 1*map_resolution_, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 2*map_resolution_, query_point(1) - 1*map_resolution_, query_point(2)));

	octomap::point3d direction(0., 0., -1.0); //查询方向: z 负方向
	vector<octomap::point3d> ground_point_set;
	octomap::point3d ground_point;
	float point_z_sum=0.0;
	bool get_ground;

	if (octo_map_->castRay(query_point, direction, ground_point, true, 10.0))
	{
		mean_ = ground_point(2);
		// cout << "mean detect: "<< mean_ << endl;
		return true;
	}
	
	for (auto it=sreach_area.begin(); it != sreach_area.end(); it++)
	{	
		get_ground = octo_map_->castRay(*it, direction, ground_point, true, 10.0);
		if(!get_ground){
			continue;
		}else
		{
			ground_point_set.push_back(ground_point);
			point_z_sum += ground_point(2);
		}
	}

	if(ground_point_set.size() < 3){
		return false;
	}else
	{
		mean_ = point_z_sum / ground_point_set.size();
		// cout << "mean detect: "<< mean_ << endl;
		return true;
	}
	
}


// >>>>>>>>>>>>>>>>>>>>>>>>>> class end <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_generator");
	ros::NodeHandle n;

	ros::spin();
}