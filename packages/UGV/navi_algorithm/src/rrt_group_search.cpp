/**************************************************************************
 * rrt_group_search.cpp
 * 
 * @Author： bornchow
 * @Date: 2022.02.11
 * 
 * @Description:
 *  本程序为rrt系列路径搜索算法的启动文件，提供以下功能：
 * 1. 选取rrt算法类型
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
#include "RRTSearch.hpp"
#include "RRTConnectSearch.hpp"
#include "RRTStarSearch.hpp"

class RRTStart
{
private:
    ros::NodeHandle n;
    ros::NodeHandle n_;

    ros::Subscriber global_map_sub_;
    ros::Subscriber goal_sub_;

    ros::Publisher rrt_node_pub_;
    ros::Publisher rrt_tree_pub_;
    ros::Publisher rrt_final_tree_pub_;

    // ros parm
    int rrt_algorithm_;
    float eta_;
    int iter_step_;
    int iter_max_;
    float search_radius_;

    nav_msgs::OccupancyGrid mapData_;
    visualization_msgs::Marker points_goal_, points_, line_, final_line_;

    // rrt algorithm
    shared_ptr<RRTGroupSearch> rrt_group_search_;

private:
    void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg);

public:
    RRTStart(ros::NodeHandle nh, ros::NodeHandle nh_);
    void visRRTProcessThread();
    void run();
    ~RRTStart();
};

RRTStart::RRTStart(ros::NodeHandle nh, ros::NodeHandle nh_) :
n(nh),
n_(nh_)
{
    //ros 参数
    nh_.param<int>("rrt_algorithm", rrt_algorithm_, 2); // 选择rrt算法
    nh_.param<float>("eta", eta_, 0.3); //rrt每次扩展时，延伸的长度
    nh_.param<int>("iter_step", iter_step_, 3100); // 为了更好的显示，每隔iter_step步，需要键入值以继续
    nh_.param<int>("iter_max", iter_max_, 3000); // 最大迭代次数[rrt-star参数]
    nh_.param<float>("search_radius", search_radius_, 2); //rewire rrt 树时的搜索半径 [rrt-star参数]

    // ros订阅与发布
    global_map_sub_ = n.subscribe("/gridMap", 100, &RRTStart::mapCallBack, this);
    goal_sub_ = n.subscribe("/clicked_point", 100, &RRTStart::goalCallBack, this);

    rrt_node_pub_ = nh.advertise<visualization_msgs::Marker>("rrt_node", 10);
    rrt_tree_pub_ = nh.advertise<visualization_msgs::Marker>("rrt_tree", 10);
    rrt_final_tree_pub_ = nh.advertise<visualization_msgs::Marker>("rrt_final_tree", 100);


    // wait until map is received, when a map is received, mapData.header.seq will not be < 1  
    while (mapData_.header.seq<1 or mapData_.data.size()<1){
        ROS_INFO("waiting for map !");
        ros::spinOnce();  
        ros::Duration(0.1).sleep();
    }

    //定义可视化参数

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

    points_.header.frame_id = mapData_.header.frame_id;
    points_.header.stamp=ros::Time(0);
    points_.ns = "points";
    points_.id = 0;
    points_.type=points_.POINTS;
    points_.action=points_.ADD;
    points_.pose.orientation.w = 1.0;
    points_.scale.x=mapData_.info.resolution; 
    points_.scale.y=mapData_.info.resolution;
    points_.color.r = 0.0/255.0;
    points_.color.g = 0.0/255.0;
    points_.color.b = 255.0/255.0;
    points_.color.a=1.0;
    points_.lifetime = ros::Duration();


    line_.header.frame_id=mapData_.header.frame_id;
    line_.header.stamp=ros::Time(0);
    line_.ns = "lines";
    line_.id = 1;
    line_.type=line_.LINE_LIST;
    line_.action=line_.ADD;
    line_.pose.orientation.w = 1.0;
    line_.scale.x = 0.03;
    line_.scale.y= 0.03;
    line_.color.r =155.0/255.0;
    line_.color.g= 155.0/255.0;
    line_.color.b =155.0/255.0;
    line_.color.a = 1.0;
    line_.lifetime = ros::Duration();

    final_line_.header.frame_id=mapData_.header.frame_id;
    final_line_.header.stamp=ros::Time(0);
    final_line_.ns = "line_back";
    final_line_.id = 1;
    final_line_.type=final_line_.LINE_LIST;
    final_line_.action=final_line_.ADD;
    final_line_.pose.orientation.w = 1.0;
    final_line_.scale.x = 0.04;
    final_line_.scale.y = 0.04;
    final_line_.color.r =255.0/255.0;
    final_line_.color.g= 0.0/255.0;
    final_line_.color.b =0.0/255.0;
    final_line_.color.a = 1.0;
    final_line_.lifetime = ros::Duration();


    switch (rrt_algorithm_)
    {
    case 0:{
        rrt_group_search_ = make_shared<RRTSearch>(eta_, iter_step_);
        break;
    }
    case 1:{
        rrt_group_search_ = make_shared<RRTConnectSearch>(eta_, iter_step_);
        break;
    }
    case 2:{
        rrt_group_search_ = make_shared<RRTStarSearch>(eta_, iter_step_, iter_max_, search_radius_);
        break;
    }

    default:
        break;
    }


}

RRTStart::~RRTStart()
{
}

void RRTStart::visRRTProcessThread(){

    ros::Rate rate(10); 
    geometry_msgs::Point p_temp;
    
    std::map<int, std::queue<geometry_msgs::Point> > iter_vis_info;
    std::queue<geometry_msgs::Point> p_final_que;

    int count;

    while (ros::ok())
    {
        rrt_group_search_->getVisInfo(iter_vis_info, p_final_que, count);

        if (count > iter_max_ - 2)
        {
            break;
        }
        

        if (count == 0) //迭代未开始
        {
            continue;
        }else
        {
            for (auto it : iter_vis_info)
            {
                points_.points.push_back(it.second.front());
                it.second.pop();
                rrt_node_pub_.publish(points_);
                while (!it.second.empty())
                {
                    line_.points.push_back(it.second.front());
                    it.second.pop();
                    line_.points.push_back(it.second.front());
                    it.second.pop();
                    rrt_tree_pub_.publish(line_);
                }
                    
            }
            

            if (rrt_group_search_->getSearchState())
            {
                // 发布最后的rrt 
                if (!p_final_que.empty())
                {
                    std::cout << "final line size: " << final_line_.points.size() << std::endl;
                    final_line_.points.clear();
                    std::cout << "final line size after: " << final_line_.points.size() << std::endl;
                    rrt_final_tree_pub_.publish(final_line_);

                    final_line_.header.frame_id=mapData_.header.frame_id;
                    final_line_.header.stamp=ros::Time(0);
                    final_line_.ns = "line_back";
                    final_line_.id = count;
                    final_line_.type=final_line_.LINE_LIST;
                    final_line_.action=final_line_.ADD;
                    final_line_.pose.orientation.w = 1.0;
                    final_line_.scale.x = 0.04;
                    final_line_.scale.y = 0.04;
                    final_line_.color.r =255.0/255.0;
                    final_line_.color.g= 0.0/255.0;
                    final_line_.color.b =0.0/255.0;
                    final_line_.color.a = 1.0;
                    final_line_.lifetime = ros::Duration();

                    std::cout << " --------- " << p_final_que.size() << std::endl;

                    p_temp = p_final_que.front();
                    p_final_que.pop();

                    while (!p_final_que.empty())
                    {
                        final_line_.points.push_back(p_temp);
                        p_temp = p_final_que.front();
                        final_line_.points.push_back(p_temp);
                        p_final_que.pop();

                        rrt_final_tree_pub_.publish(final_line_);
                        sleep(0.1);

                    }
                    std::cout << "final line size add: " << final_line_.points.size() << std::endl;
                }else{
                    continue;
                }
                
                //break;
            }
            
        }


        rate.sleep();
    }
    
}

void RRTStart::run(){

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
        
        rrt_node_pub_.publish(points_goal_);
        ros::spinOnce();
    }

    //加载地图
    rrt_group_search_->updateMap(mapData_);
    ROS_INFO("map info origin [%f, %f]  -- size [%d,  %d]", 
        mapData_.info.origin.position.x, 
        mapData_.info.origin.position.y, 
        mapData_.info.width, mapData_.info.height);
    

    geometry_msgs::Point p_start, p_end;
    p_start.x = points_goal_.points[0].x; p_start.y = points_goal_.points[0].y;
    p_end.x = points_goal_.points[1].x; p_end.y = points_goal_.points[1].y;
    rrt_group_search_->search(p_start, p_end);


}

void RRTStart::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    mapData_ = *msg;
}

void RRTStart::goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg){
    points_goal_.points.push_back(msg->point);
}



//// class end;
int main(int argc, char** argv){

    ros::init(argc, argv, "rrt_group_search");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate rate(10);
    
    RRTStart rrt_start(nh, nh_);

    std::thread visRRTThread(&RRTStart::visRRTProcessThread, &rrt_start);

    rrt_start.run();

    visRRTThread.join();


}
