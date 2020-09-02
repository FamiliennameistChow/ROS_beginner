 /********************************************************
 * path_planing.cpp
 * path_planing use octomap method
 * 
 * Author： Born Chow
 * Date: 2020.06.21
 * 
 * 说明:在octomap上实现rrt导航,用于小车的起伏地形导航
 * 使用pair描述rrt树，运算时间较长
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

using namespace std;
typedef vector<pair<octomap::point3d, octomap::point3d> > Point_pair_set; //储存(当前节，父节点)对
// Declear some global variables

//ROS publishers
ros::Publisher vis_pub;
ros::Publisher traj_pub;
ros::Publisher line_pub;
ros::Publisher line_back_pub;


class planner
{
private:
	vector<octomap::point3d> rrt_path;
	Point_pair_set rrt_tree_set;  
	// Flag for initialization
	bool set_start = false;
	float map_resolution;
	geometry_msgs::Point start_point, goal_point, pre_goal_point;
	shared_ptr<octomap::OcTree> map_tree;
	octomap::point3d p_new, p_rand, p_nearest;

	// for vis
	visualization_msgs::Marker marker, line, line_back;
	int nu;

	//function 
	float Norm(octomap::point3d p, octomap::point3d p1);
	void backTree(octomap::point3d p, octomap::point3d p_init, vector<octomap::point3d> &rrt_tree, float &cost);
	void backTree(octomap::point3d p, octomap::point3d p_init, float &cost);
	void eraseNode(Point_pair_set &V, octomap::point3d p);
	// rrt function
	octomap::point3d Near(Point_pair_set &V ,octomap::point3d p);
	octomap::point3d Steer(octomap::point3d nearest_point, octomap::point3d rand_point, float eta_);
	int collisionFree(octomap::point3d nearest_point, octomap::point3d new_point);
	bool isGoal(octomap::point3d new_point, octomap::point3d end_point);
	void rewirteTree(Point_pair_set &V, octomap::point3d new_point, octomap::point3d nearest_point, float search_r);
	// UGV trajectory
	trajectory_msgs::MultiDOFJointTrajectory msg;
	trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;

	bool point_equal(double x, double y, double z, geometry_msgs::Point point){
		if(abs(point.x - x)< 0.00001 && abs(point.y - y)< 0.00001 && abs(point.z - z)< 0.00001){
			return true;
		}else
		{
			return false;
		}	
	}

	bool point_equal(octomap::point3d p, octomap::point3d p1){
		float half_szie = map_resolution / 2; 
		if(abs(p(0) - p1(0)) <= half_szie && abs(p(1) - p1(1)) <= half_szie && abs(p(2) - p1(2)) <= half_szie){
			return true;
		}else
		{
			return false;
		}
	}

	double getRandData(int min,int max)
	{
		double m1=(double)(rand()%101)/101; // 计算 0，1之间的随机小数,得到的值域近似为(0,1)
		min++;  //将 区间变为(min+1,max),
		double m2=(double)((rand()%(max-min+1))+min); //计算 min+1,max 之间的随机整数，得到的值域为[min+1,max]
		m2=m2-1; //令值域为[min,max-1]
		return m1+m2;  //返回值域为(min,max),为所求随机浮点数
	}

public:

	planner(void);
	~planner();
	void init_start(void);
	void setStart(double x, double y, double z);
	void setGoal(double x, double y, double z);
	void plan(void);
	void replan(void);
	void updateMap(shared_ptr<octomap::OcTree> map);
	bool validGround(octomap::point3d query_point);
	bool validGround(octomap::point3d query_point, float& mean_);

	octomap::point3d carBodySize;
	float th_stdev;
	
};

planner::planner(void)
{
	//init start_point
	start_point.x = 0;
	start_point.y = 0;
	start_point.z = 0;

	//平整度阈值
	th_stdev = 0.1;

	// load car size
	carBodySize(0) = 0.6;  //x
	carBodySize(1) = 0.8;  //y
	carBodySize(2) = 0.8;  //z  轮底到激光雷达的距离

	init_start(); //测试用

	//init rrt 规划计数
	nu = 0;

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
    line_back.header.frame_id= "map";
    line_back.header.stamp=ros::Time(0);
    line_back.ns = "line_back";
    line_back.id = 2;
    line_back.type=line.LINE_LIST;
    line_back.action=line.ADD;
    line_back.pose.orientation.w = 1.0;
    line_back.scale.x = 0.12;
    line_back.scale.y= 0.12;
    line_back.color.r =255.0/255.0;
    line_back.color.g= 0.0/255.0;
    line_back.color.b =0.0/255.0;
    line_back.color.a = 1.0;
    line_back.lifetime = ros::Duration();

 
	cout<<"[planner] Initialized"<<endl;


}

planner::~planner()
{
}

void planner::init_start(void)
{
	if(!set_start)
		std::cout << "[planner] Start point set " << std::endl;
	set_start = true;
}

void planner::setGoal(double x, double y, double z){
	if (!point_equal(x,y,z, pre_goal_point))
	{
		goal_point.x = x;
		goal_point.y = y;
		goal_point.z = z;
		pre_goal_point = goal_point;

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
		vis_pub.publish(marker);
		// vis

		cout << "[planner] Goal point set to: " << x << " " << y << " " << z << std::endl;
		if(set_start){
			plan();
		}else
		{
			cout<<"[planner] no Start point set !!"<< endl;
		}
		
	}
	
}


void planner::setStart(double x, double y, double z){
	start_point.x = x;
	start_point.y = y;
	start_point.z = z;
}

void planner::updateMap(shared_ptr<octomap::OcTree> map){
		map_tree = map;
		map_resolution = map_tree->getResolution();
		cout<<"map Resolution: "<< map_resolution <<endl;
}

// process octomap
bool planner::validGround(octomap::point3d query_point){
	// sreach the ground state
	// sreach pattern
	//     *
	//   *****
	//  ***#***
	//   *****
	//     * 
	vector <octomap::point3d> sreach_area;
	sreach_area.push_back(query_point);
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 3*map_resolution, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 3*map_resolution, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) + 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) + 2*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) - 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) - 2*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution, query_point(1) + 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution, query_point(1) - 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution, query_point(1) + 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution, query_point(1) - 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution, query_point(1) + 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution, query_point(1) - 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 2*map_resolution, query_point(1) + 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 2*map_resolution, query_point(1) - 1*map_resolution, query_point(2)));

	octomap::point3d direction(0., 0., -1.0); //查询方向: z 负方向
	vector<octomap::point3d> ground_point_set;
	octomap::point3d ground_point;
	float point_z_sum=0.0;
	bool get_ground;

	for (auto it=sreach_area.begin(); it != sreach_area.end(); it++)
	{	
		get_ground = map_tree->castRay(*it, direction, ground_point, true, 10.0);
		// cout << "search " << *it << endl;
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
		float mean = point_z_sum / ground_point_set.size();
		double accum  = 0.0;
		for(auto it=ground_point_set.begin(); it != ground_point_set.end(); it++){
			accum += ((*it)(2) - mean)*((*it)(2) - mean);
		}
		float stdev = sqrt(accum/(ground_point_set.size()-1));
		if(stdev > th_stdev){ //0.005 0.1
			// cout<< query_point <<  " stdev: " << stdev <<endl; 
			return false;
		}
	}
	return true;

}

// 返回地形高度，用于显示
bool planner::validGround(octomap::point3d query_point, float& mean_){
	// sreach the ground state
	// sreach pattern
	// 
	//   *****
	//   **#**
	//   *****
	//
	     
	vector <octomap::point3d> sreach_area;
	sreach_area.push_back(query_point);
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution, query_point(1)                   , query_point(2)));
	// sreach_area.push_back(octomap::point3d(query_point(0) + 3*map_resolution, query_point(1)                   , query_point(2)));
	// sreach_area.push_back(octomap::point3d(query_point(0) - 3*map_resolution, query_point(1)                   , query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) + 1*map_resolution, query_point(2)));
	// sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) + 2*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) - 1*map_resolution, query_point(2)));
	// sreach_area.push_back(octomap::point3d(query_point(0)                   , query_point(1) - 2*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution, query_point(1) + 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 1*map_resolution, query_point(1) - 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution, query_point(1) + 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) + 2*map_resolution, query_point(1) - 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution, query_point(1) + 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 1*map_resolution, query_point(1) - 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 2*map_resolution, query_point(1) + 1*map_resolution, query_point(2)));
	sreach_area.push_back(octomap::point3d(query_point(0) - 2*map_resolution, query_point(1) - 1*map_resolution, query_point(2)));

	octomap::point3d direction(0., 0., -1.0); //查询方向: z 负方向
	vector<octomap::point3d> ground_point_set;
	octomap::point3d ground_point;
	float point_z_sum=0.0;
	bool get_ground;

	if (map_tree->castRay(query_point, direction, ground_point, true, 10.0))
	{
		mean_ = ground_point(2);
		// cout << "mean detect: "<< mean_ << endl;
		return true;
	}
	
	for (auto it=sreach_area.begin(); it != sreach_area.end(); it++)
	{	
		get_ground = map_tree->castRay(*it, direction, ground_point, true, 10.0);
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

float planner::Norm(octomap::point3d p1, octomap::point3d p2){
	//　返回两点的欧式距离(x,y方向)
	return pow( pow(p1(0)-p2(0), 2) + pow(p1(1)-p2(1),2), 0.5);

}

// 计算rrt_tree上离随机点最近的点
octomap::point3d planner::Near(Point_pair_set &V ,octomap::point3d p){
	float min = Norm(V[0].first, p);
    float temp;
    octomap::point3d nearest_point;

    for (auto vp : V)
    {
        temp = Norm(vp.first, p);
        if(temp <=min )
        {
            min = temp;
            nearest_point = vp.first;
        }
    }
    return nearest_point;
}


octomap::point3d planner::Steer(octomap::point3d nearest_point, octomap::point3d rand_point, float eta_){
	octomap::point3d new_point;
    float dis = Norm(nearest_point, rand_point);
    if(dis <= eta_)
    {
        new_point = rand_point;
    }else{
        new_point(0) = nearest_point(0) + eta_ * (rand_point(0) - nearest_point(0)) / dis;
        new_point(1) = nearest_point(1) + eta_ * (rand_point(1) - nearest_point(1)) / dis;
    }

    return new_point;
	

}

// 地面状态,  0: 无地面信息　1:有地面信息
int planner::collisionFree(octomap::point3d nearest_point, octomap::point3d new_point){
	float step_length = map_resolution*3; //*2
	int step_nu = ceil(Norm(nearest_point, new_point)/step_length);  //ceil(x)返回的是大于x的最小整数
	octomap::point3d step_point = nearest_point;
	int state;
	for (size_t i = 0; i < step_nu; i++)
	{
		step_point = Steer(step_point, new_point, step_length);
		if(validGround(step_point)){
			state = 1;
			continue;
		}else
		{
			state = 0;
			return state;
		}
	}
	return state;
	
}

bool planner::isGoal(octomap::point3d new_point, octomap::point3d end_point){
    if(Norm(new_point, end_point) < map_resolution*6)
    {
        return true;
    }else
    {
        return false;
    }
    
}

// 遍历rrt, 返回查询节点到起始节点的所有父节点, 以及查询节点代价
void planner::backTree(octomap::point3d p, octomap::point3d p_init, vector<octomap::point3d> &rrt_tree, float &cost){
	octomap::point3d point_back;
	point_back = p;
	cost = 0.0;
	for (size_t i = 0; i < rrt_tree_set.size(); i++)
	{
		for (auto it = rrt_tree_set.begin(); it != rrt_tree_set.end(); it++)
		{
			if (point_equal((*it).first, point_back))
			{
				cost += Norm(point_back, (*it).second);
				rrt_tree.push_back(point_back);
				point_back = (*it).second;
				if (point_equal(point_back, p_init))
				{
					rrt_tree.push_back(point_back);
					return;
				}
				break;
			}
			
		}
		
	}	
}

// 遍历rrt, 查询节点到起始节点的所有父节点, 返回查询节点代价
void planner::backTree(octomap::point3d p, octomap::point3d p_init, float &cost){
	octomap::point3d point_back;
	point_back = p;
	cost = 0.0;
	for (size_t i = 0; i < rrt_tree_set.size(); i++)
	{
		for (auto it = rrt_tree_set.begin(); it != rrt_tree_set.end(); it++)
		{
			if (point_equal((*it).first, point_back))
			{
				cost += Norm(point_back, (*it).second);
				point_back = (*it).second;
				if (point_equal(point_back, p_init))
				{
					return;
				}
				break;
			}
		}	
	}
}

void planner::eraseNode(Point_pair_set &V, octomap::point3d p){
	for(auto iter=V.begin();iter!=V.end();iter++){        //从vector中删除指定的某一个元素 
		if(point_equal((*iter).first, p)){
			V.erase(iter);
			break;
		}
	}
}

void planner::rewirteTree(Point_pair_set &V, octomap::point3d new_point, octomap::point3d nearest_point, float search_r){
	vector<octomap::point3d> potential_parent;
	vector<float> cost_list;
	multimap<float, octomap::point3d> cost_point_pair;  //键值不能重复，相同cost如何处理-> 使用multimap //代价-节点
	float cost;
	float nearest_point_cost, min_cost, new_cost;
	float dis, dis_cost;
	int check;

	backTree(nearest_point, V[0].first, nearest_point_cost);
	min_cost = nearest_point_cost + Norm(nearest_point, new_point);
	// search near node 使用multimap排序
	for (auto vp : V)
	{
		dis = Norm(vp.first, new_point);
		if( dis<= search_r){
			potential_parent.push_back(vp.first);
			backTree(vp.first, V[0].first, cost);
			// cost += dis; //加上该点到新节点的距离
			cost_list.push_back(cost);
			cost_point_pair.insert(make_pair(cost, vp.first));
		}
	}
	//choose parent 排序处理
	for (multimap<float, octomap::point3d>::iterator it = cost_point_pair.begin(); it != cost_point_pair.end(); it++)
	{
		dis_cost = (*it).first + Norm((*it).second, new_point);
		if (dis_cost == min_cost)
		{
			V.push_back(make_pair(p_new, nearest_point));
			return;
		}
		
		if (dis_cost < min_cost)
		{
			check = collisionFree((*it).second, new_point);
			if (check == 0)
			{
				continue;
			}
			if (check == 1)
			{
				V.push_back(make_pair(p_new, (*it).second));
				new_cost = dis_cost;
				for(auto iter=cost_point_pair.begin(); iter != cost_point_pair.end(); iter++){
					if (new_cost + Norm((*iter).second, new_point) < (*iter).first)
					{
						eraseNode(V, (*iter).second);
						V.push_back(make_pair((*iter).second, new_point));
					}
					
				}
				return;
			}
	
		}
		
	}

}


void planner::plan(void){
	cout<<"start plan"<<endl;
	octomap::point3d base_start(start_point.x, start_point.y, start_point.z);
	octomap::point3d base_goal(goal_point.x, goal_point.y, goal_point.z);


	// octomap::point3d base(1.86, 3.8, 3);
	// octomap::point3d direction(0., 0., -1.0); //查询方向: z 负方向
	// cout<<"base_start :" << base_start << endl;

	// cout << "point " << base_start << " has valid gound info: " << validGround(base_start) << endl;
	// cout << "point " << base << " has valid gound info: " << validGround(base) << endl;

	// octomap::point3d ground_point;
	// bool has_ground;
	// has_ground = map_tree->castRay(base, direction, ground_point, true, 5.0);
	// cout << "point " << base << " has gound: " << has_ground << "  ground point: "<<ground_point << endl;
	rrt_tree_set.clear();
	line_back.points.clear();
	line.points.clear();
	rrt_path.clear();
	rrt_tree_set.push_back(make_pair(base_start, base_start));

	int a;
	int checking;

	float eta = 2.0; // 1.5
	float search_near = 4.0;
	geometry_msgs::Point point_pub;
	float mean;
	double minX, minY, minZ, maxX, maxY, maxZ;
	map_tree->getMetricMin(minX, minY, minZ);
	map_tree->getMetricMax(maxX, maxY, maxZ);
	clock_t time_start=clock();


	while (ros::ok())
	{
		if(nu % 30000 == 0)
        {
			cout <<"input any to continue: " << endl;
            a = cin.get();  
        }
		cout<<" -----------------------"<< endl;
		cout << "rand smaple a point " << endl;

		if (nu % 8 == 0)
		{
			p_rand(0) = getRandData(base_goal(0) - 5, base_goal(0) + 5);
			p_rand(1) = getRandData(base_goal(1) - 5, base_goal(1) + 5);
		}else
		{
			p_rand(0) = getRandData(minX, maxX);
			p_rand(1) = getRandData(minY, maxY);
		}
		
		

		// cout << "rand point-- " << p_rand << endl;

		p_nearest = Near(rrt_tree_set, p_rand);
		// cout << "nearest point-- " << p_nearest << endl;

		p_new = Steer(p_nearest, p_rand, eta);
        // cout << "new point-- " << p_new << endl;
		// 计算采样点地面数据
		p_new(2) = 5;
		// cout << validGround(p_new, mean) << endl;
		if(validGround(p_new, mean)){
			p_new(2) = mean + carBodySize(2);
		}else
		{
			p_new(2) = p_nearest(2);
		}
		// cout << "new point　detect ground-- " << p_new << endl;

		// vis 显示采样点
		marker.id = nu;
		marker.ns = "path";
		marker.scale.x = 0.15;
		marker.scale.y = 0.15;
		marker.scale.z = 0.15;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.pose.position.x = p_new(0);
		marker.pose.position.y = p_new(1);
		marker.pose.position.z = p_new(2);
		vis_pub.publish(marker);
		// vis

		// 地面状态,  0: 无地面信息　1:有地面信息
		checking = collisionFree(p_nearest, p_new);

		if (checking == 0)
		{
			cout<<"[rrt plan]: collosion" << endl;
			// vis 显示阻断的节点 测试用
			marker.ns = "path";
			marker.id = nu;
			marker.scale.x = 0.15;
			marker.scale.y = 0.15;
			marker.scale.z = 0.15;
			marker.color.a = 1.0;
			marker.color.r = 255.0/255.0;
			marker.color.g = 0.0/255.0;
			marker.color.b = 255.0  /255.0;
			marker.pose.position.x = p_new(0);
			marker.pose.position.y = p_new(1);
			marker.pose.position.z = p_new(2);
			vis_pub.publish(marker);
			// vis 显示阻断的节点
		}
		if (checking == 1)
		{
			cout <<"[rrt plan]: rrt tree add new point " <<endl;

			// rrt-star
			rewirteTree(rrt_tree_set, p_new, p_nearest, search_near);
			// rrt
			// rrt_tree_set.push_back(make_pair(p_new, p_nearest));

			// vis 线段表示 rrt tree
			point_pub.x = p_new(0);
			point_pub.y = p_new(1);
			point_pub.z = p_new(2);
			line.points.push_back(point_pub);
			point_pub.x = p_nearest(0);
			point_pub.y = p_nearest(1);
			point_pub.z = p_nearest(2);
			line.points.push_back(point_pub);
			line_pub.publish(line);
			// vis
			
			if (isGoal(p_new, base_goal))
			{	
				octomap::point3d p_back, p_pub;
				rrt_tree_set.push_back(make_pair(base_goal, p_new));
				cout << "[rrt plan]: you have got the goal!---- " <<endl;
				clock_t time_end=clock();
				cout<<"[rrt plan]: time use:"<<1000*(time_end-time_start)/(double)CLOCKS_PER_SEC<<"ms"<<endl;

				// trajectory pub set
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = "world";
				msg.joint_names.clear();
				msg.points.clear();
				msg.joint_names.push_back("ugv");

				//rrt tree 回溯
				float cost_total = 0.0;
				backTree(base_goal, base_start, rrt_path, cost_total);
				reverse(rrt_path.begin(), rrt_path.end()); //反转vector

				cout<<"[rrt plan]: path size: " << rrt_path.size() << endl;
				for (size_t i = 0; i < rrt_path.size(); i++)
				{
					p_pub = rrt_path[i];
					cout<<"[rrt plan]: path size: " << rrt_path.size() << endl;
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

					if(i+1 == rrt_path.size()){
						continue;
					}
					//vis
					cout<<"rrt path first point: " << p_pub <<endl;
					point_pub.x = p_pub(0);
					point_pub.y = p_pub(1);
					point_pub.z = p_pub(2);
					line_back.points.push_back(point_pub);
					p_pub= rrt_path[i+1];
					cout<<"rrt path second point: " << p_pub <<endl;
					point_pub.x = p_pub(0);
					point_pub.y = p_pub(1);
					point_pub.z = p_pub(2);
					line_back.points.push_back(point_pub);
					line_back_pub.publish(line_back);
					//vis
				}

				traj_pub.publish(msg);



				time_end=clock();
				cout<<"[rrt plan]: total time use:"<<1000*(time_end-time_start)/(double)CLOCKS_PER_SEC<<"ms"<<endl;

				// //rrt tree 回溯
				// cout <<"[rrt plan]: opti input any to continue: " << endl;
				// a = cin.get();
				// line_back.points.clear();

				break;
			}
			
		}
		nu++;

	}
	
}

void planner::replan(void){
	// plan();
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>class end<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, planner* planner_ptr)
{

	// convert octree to collision object
	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
	planner_ptr->updateMap(shared_ptr<octomap::OcTree>(tree_oct));
	planner_ptr->replan();
}

void odomCb(const nav_msgs::Odometry::ConstPtr &msg, planner* planner_ptr)
{
	planner_ptr->setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	planner_ptr->init_start();
}

void startCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner* planner_ptr)
{
	planner_ptr->setStart(msg->point.x, msg->point.y, msg->point.z);
	planner_ptr->init_start();
}

void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner* planner_ptr)
{
	planner_ptr->setGoal(msg->point.x, msg->point.y, msg->point.z);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "octomap_planner");
	ros::NodeHandle n;
	planner planner_object;

	ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, boost::bind(&octomapCallback, _1, &planner_object));
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/scout/odometry", 1, boost::bind(&odomCb, _1, &planner_object));
	ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, boost::bind(&goalCb, _1, &planner_object));
	// ros::Subscriber start_sub = n.subscribe<geometry_msgs::PointStamped>("/start/clicked_point", 1, boost::bind(&goalCb, _1, &planner_object));

	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 ); // 发布采样点
	traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",1);  // 发布最终轨迹
	line_pub = n.advertise<visualization_msgs::Marker>("lines", 10); //发布rrt 树
    line_back_pub = n.advertise<visualization_msgs::Marker>("lines_back", 10);	//发布回溯rrt节点
	

	ros::spin();

	return 0;
}
