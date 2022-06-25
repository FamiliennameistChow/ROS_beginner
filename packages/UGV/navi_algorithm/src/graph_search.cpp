/**************************************************************************
 * graph_search.cpp
 * 
 * @Author： bornchow
 * @Date: 2022.06.18
 * 
 * @Description:
 *  本程序为图搜索系列路径搜索算法的启动文件，提供以下功能：
 * 1. 选取图搜索算法类型
 * 2. 支持算法可视化(rviz)
 * 3. 与rviz交互,包括获取地图信息与起始点信息
 *  
 *  ****************************************************/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include <iostream>
#include <vector> 
#include <thread>
#include "AStarSearch.hpp"
#include "DijkstraSearch.hpp"
#include "JPSSearch.hpp"

using namespace std;

class GraphSearch
{
private:
    ros::NodeHandle n;

    ros::Subscriber global_map_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher graph_node_pub_;
    ros::Publisher final_path_pub_;

    // ros param
    int graph_sreach_algorithm_;
    int heuristic_func_;
    int iter_step_;

    nav_msgs::OccupancyGrid mapData_;

    // for vis
    visualization_msgs::Marker points_goal_, points_; 
    nav_msgs::Path final_path_;

    shared_ptr<GraphGroupSearch> graph_sreach_;
private:
    void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg);
public:
    GraphSearch(ros::NodeHandle nh, ros::NodeHandle nh_);
    ~GraphSearch();
    void run();
    void visGraphSreachProcessThread();
};

GraphSearch::GraphSearch(ros::NodeHandle nh, ros::NodeHandle nh_):
n(nh)
{
    // ros参数
    nh_.param<int>("graph_sreach_algorithm", graph_sreach_algorithm_, 0);
    nh_.param<int>("heuristic_func", heuristic_func_, 0);
    nh_.param<int>("iter_step", iter_step_, 2); // 为了更好的显示，每隔iter_step步，需要键入值以继续

    //ros 订阅与发布
    global_map_sub_ = n.subscribe("/gridMap", 10, &GraphSearch::mapCallBack, this);
    goal_sub_ = n.subscribe("/clicked_point", 10, &GraphSearch::goalCallBack, this);

    graph_node_pub_ = n.advertise<visualization_msgs::Marker>("graph_node", 10);
    final_path_pub_ = n.advertise<nav_msgs::Path>("final_path", 10);

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

    // 图搜索节点可视化
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

    // 最终路径
    final_path_.header.frame_id = mapData_.header.frame_id;
    final_path_.header.stamp = ros::Time(0);

    // 定义图搜索算法
    switch (graph_sreach_algorithm_)
    {
    case 0:{
        graph_sreach_ = make_shared<DijkstraSearch>(iter_step_);
        break;
    }
    case 1:{
        graph_sreach_ = make_shared<AStarSearch>(heuristic_func_, iter_step_);
        break;
    }
    case 2:{
        graph_sreach_ = make_shared<JPSSearch>(heuristic_func_, iter_step_);
    }
        
    default:

        break;
    }
    

}

GraphSearch::~GraphSearch()
{
}

void GraphSearch::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    mapData_ = *msg;
}

void GraphSearch::goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg){
    points_goal_.points.push_back(msg->point);
}

void GraphSearch::run(){

    // 获取起始点
    double last_command = ros::Time::now().toSec();
    ros::Rate loop_rate(20);
    while (points_goal_.points.size()<3)
    {
        double now = ros::Time::now().toSec();
        if(points_goal_.points.size() == 0){
            ROS_INFO("set {start point} in rviz");
        }

        if(points_goal_.points.size() == 1){
            ROS_INFO("set {goal point} in rviz");
            graph_node_pub_.publish(points_goal_);
        }

        if(points_goal_.points.size() == 2){
            ROS_INFO("clicked point to start!!!");
            graph_node_pub_.publish(points_goal_);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    graph_sreach_->updateMap(mapData_);
    ROS_INFO("map info origin [%f, %f]  -- size [%d,  %d] -- re: [%f]", 
        mapData_.info.origin.position.x, 
        mapData_.info.origin.position.y, 
        mapData_.info.width, mapData_.info.height, mapData_.info.resolution);
    
    geometry_msgs::Point p_start, p_end;
    p_start.x = points_goal_.points[0].x; p_start.y = points_goal_.points[0].y;
    p_end.x = points_goal_.points[1].x; p_end.y = points_goal_.points[1].y;
    graph_sreach_->search(p_start, p_end);
    
    
}


void GraphSearch::visGraphSreachProcessThread()
{

    ros::Rate rate(20);
    
    std::map<int, std::queue<geometry_msgs::Point> > iter_vis_info;
    std::queue<geometry_msgs::Point> p_final_que;

    int count;

    while (ros::ok())
    {
        // std::cout << " in vis..." << std::endl;
        graph_sreach_->getVisInfo(iter_vis_info, p_final_que, count);

        if (!iter_vis_info.empty())
        {
            for(auto it: iter_vis_info){

                while (!it.second.empty())
                {
                    points_.points.push_back(it.second.front());
                    it.second.pop();
                }
                graph_node_pub_.publish(points_);
            }
            iter_vis_info.clear();
        }

        if (graph_sreach_->getSearchState())
        {
            //发布最终节点
            while (!p_final_que.empty())
            {
                geometry_msgs::PoseStamped thisPose;
                thisPose.header.frame_id = mapData_.header.frame_id;
                thisPose.header.stamp = ros::Time::now();
                thisPose.pose.position = p_final_que.front();
                thisPose.pose.orientation.x = 0;
                thisPose.pose.orientation.y = 0;
                thisPose.pose.orientation.z = 0;
                thisPose.pose.orientation.w = 1;
                p_final_que.pop();
                final_path_.poses.push_back(thisPose);
                final_path_pub_.publish(final_path_);
            }

        }
        
        final_path_pub_.publish(final_path_);
        ros::spinOnce();
        rate.sleep();
        
    }
    
}

///////////////////////////////////////////

int main(int argc, char** argv){

    ros::init(argc, argv, "graph_search_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate rate(10);
    
    GraphSearch g_search(nh, nh_);

    std::thread visAThread(&GraphSearch::visGraphSreachProcessThread, &g_search);

    g_search.run();

    visAThread.join();
}