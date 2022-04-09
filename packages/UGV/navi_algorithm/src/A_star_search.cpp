/**************************************************************************
 * A_star_search.cpp
 * 
 * @Author： bornchow
 * @Date: 2022.03.16
 * 
 * @Description:
 *  本程序为A-star系列路径搜索算法的启动文件，提供以下功能：
 * 1. 选取A-star算法类型
 * 2. 支持算法可视化(rviz)
 * 3. 与rviz交互,包括获取地图信息与起始点信息
 *  
 *  ****************************************************/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include <iostream>
#include <vector> 
#include <thread>
#include "AStarSearch.hpp"

using namespace std;

class AStart
{
private:
    ros::NodeHandle n;
    ros::NodeHandle n_;

    ros::Subscriber global_map_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher A_node_pub_;
    ros::Publisher A_final_node_pub_;

    // ros param
    int A_algorithm_;
    int heuristic_func_;
    int iter_step_;

    nav_msgs::OccupancyGrid mapData_;

    // for vis
    visualization_msgs::Marker points_goal_, points_, final_points_; 
    

    // A algorithm
    shared_ptr<AStarSearch> A_star_search_;

private:
    void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg);

public:
    AStart(ros::NodeHandle nh, ros::NodeHandle nh_);
    ~AStart();
    void visAStarProcessThread();
    void run();
};

AStart::AStart(ros::NodeHandle nh, ros::NodeHandle nh_) :
n(nh),
n_(nh_)
{
    // ros参数
    // nh_.param<int>("A_algorithm", A_algorithm_, 0);
    nh_.param<int>("heuristic_func", heuristic_func_, 0);
    nh_.param<int>("iter_step", iter_step_, 2); // 为了更好的显示，每隔iter_step步，需要键入值以继续

    //ros 订阅与发布
    global_map_sub_ = n.subscribe("/gridMap", 100, &AStart::mapCallBack, this);
    goal_sub_ = n.subscribe("/clicked_point", 100, &AStart::goalCallBack, this);

    A_node_pub_ = n.advertise<visualization_msgs::Marker>("A_star_node", 10);
    A_final_node_pub_ = n.advertise<visualization_msgs::Marker>("A_star_final_node", 1);

    // wait until map is received, when a map is received, mapData.header.seq will not be < 1  
    while (mapData_.header.seq<1 or mapData_.data.size()<1){
        ROS_INFO("waiting for map !");
        ros::spinOnce();  
        ros::Duration(0.1).sleep();
    }

    // 定义可视化参数

    // 起始点可视化
    points_goal_.header.frame_id = mapData_.header.frame_id;
    points_goal_.header.stamp=ros::Time(0);
    points_goal_.ns = "points_goal";
    points_goal_.id = 0;
    points_goal_.type=points_goal_.POINTS;
    points_goal_.action=points_goal_.ADD;
    points_goal_.pose.orientation.w = 1.0;
    points_goal_.scale.x=mapData_.info.resolution; 
    points_goal_.scale.y=mapData_.info.resolution;
    points_goal_.color.r = 0.0/255.0;
    points_goal_.color.g = 255.0/255.0;
    points_goal_.color.b = 255.0/255.0;
    points_goal_.color.a=1.0;
    points_goal_.lifetime = ros::Duration();

    // A-star节点可视化
    // 点表示扩展出来的节点
    points_.header.frame_id = mapData_.header.frame_id;
    points_.header.stamp=ros::Time(0);
    points_.ns = "points";
    points_.id = 0;
    points_.type=points_.POINTS;
    points_.action=points_.ADD;
    points_.pose.orientation.w = 1.0;
    points_.scale.x=mapData_.info.resolution; 
    points_.scale.y=mapData_.info.resolution;
    points_.color.r = 255/255.0;
    points_.color.g = 255.0/255.0;
    points_.color.b = 173.0/255.0;
    points_.color.a = 0.5;
    points_.lifetime = ros::Duration();

    // A-star最终节点可视化
    // 红色表示最终节点
    final_points_.header.frame_id = mapData_.header.frame_id;
    final_points_.header.stamp=ros::Time(0);
    final_points_.ns = "points";
    final_points_.id = 0;
    final_points_.type=final_points_.POINTS;
    final_points_.action=final_points_.ADD;
    final_points_.pose.orientation.w = 1.0;
    final_points_.scale.x=mapData_.info.resolution; 
    final_points_.scale.y=mapData_.info.resolution;
    final_points_.color.r = 255.0/255.0;
    final_points_.color.g = 0.0/255.0;
    final_points_.color.b = 0.0/255.0;
    final_points_.color.a = 1.0;
    final_points_.lifetime = ros::Duration();

    A_star_search_ = make_shared<AStarSearch>(heuristic_func_, iter_step_);
}

AStart::~AStart()
{
}

void AStart::run(){

    // 获取起始点
    double last_command = ros::Time::now().toSec();
    while (points_goal_.points.size()<3)
    {
        double now = ros::Time::now().toSec();
        if (points_goal_.points.size()<1 && now - last_command > 2)
        {
            ROS_INFO("set {start point} in rviz");
            last_command = ros::Time::now().toSec();
        }else if (points_goal_.points.size()<2 && now - last_command > 2)
        {
            ROS_INFO("set {goal point} in rviz");
            last_command = ros::Time::now().toSec();
        }else if (points_goal_.points.size()<3 && now - last_command > 2)
        {
            ROS_INFO("clicked point to start!!!");
            last_command = ros::Time::now().toSec();
        }
        
        A_node_pub_.publish(points_goal_);
        ros::spinOnce();
    }

    A_star_search_->updateMap(mapData_);
    ROS_INFO("map info origin [%f, %f]  -- size [%d,  %d]　-- re: [%f]", 
        mapData_.info.origin.position.x, 
        mapData_.info.origin.position.y, 
        mapData_.info.width, mapData_.info.height, mapData_.info.resolution);
    
    geometry_msgs::Point p_start, p_end;
    p_start.x = points_goal_.points[0].x; p_start.y = points_goal_.points[0].y;
    p_end.x = points_goal_.points[1].x; p_end.y = points_goal_.points[1].y;
    A_star_search_->search(p_start, p_end);
    
    
}

void AStart::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    mapData_ = *msg;
}

void AStart::goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg){
    points_goal_.points.push_back(msg->point);
}

void AStart::visAStarProcessThread(){

    ros::Rate rate(20);
    
    std::map<int, std::queue<geometry_msgs::Point> > iter_vis_info;
    std::queue<geometry_msgs::Point> p_final_que;

    int count;

    while (ros::ok())
    {
        A_star_search_->getVisInfo(iter_vis_info, p_final_que, count);

        if (!iter_vis_info.empty())
        {
            for(auto it: iter_vis_info){

                while (!it.second.empty())
                {
                    points_.points.push_back(it.second.front());
                    it.second.pop();
                }
                A_node_pub_.publish(points_);
            }
        }

        if (A_star_search_->getSearchState())
        {
            //发布最终节点
            while (!p_final_que.empty())
            {
                final_points_.points.push_back(p_final_que.front());
                p_final_que.pop();
                A_final_node_pub_.publish(final_points_);
            }

        }
        
        
    }
    
}


///////////////////////////////////////////

int main(int argc, char** argv){

    ros::init(argc, argv, "A_group_search");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate rate(10);
    
    AStart A_start(nh, nh_);

    std::thread visAThread(&AStart::visAStarProcessThread, &A_start);

    A_start.run();

    visAThread.join();
}