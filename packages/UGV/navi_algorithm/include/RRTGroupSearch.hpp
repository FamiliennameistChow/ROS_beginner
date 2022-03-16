
#pragma once
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include "ros/ros.h"
#include <iostream>
#include <vector>  
#include <unordered_map>
#include <map>
#include <queue>

using namespace std;

//为geometry_msgs::Point定义一个比较函数与相等函数
struct HashFunc
{
    std::size_t operator()(const geometry_msgs::Point &p) const 
    {
        using std::size_t;
        using std::hash;
 
        return ((hash<int>()(p.x)
            ^ (hash<int>()(p.y) << 1)) >> 1)
            ^ (hash<int>()(p.z) << 1);
    }
};

struct EqualKey
{
    bool operator () (const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) const
    {
        return p1.x  == p2.x
            && p1.y  == p2.y
            && p1.z  == p2.z;
    }
};

struct NodeCost{
    geometry_msgs::Point this_node;
    geometry_msgs::Point father_node;
    float cost;
};

// 子节点， 父节点
typedef unordered_map<geometry_msgs::Point, geometry_msgs::Point, HashFunc, EqualKey> PointPair;
// 子节点， {父节点， 代价}
typedef unordered_map<geometry_msgs::Point, NodeCost, HashFunc, EqualKey> NodeCostPair;

// 基类定义

class RRTGroupSearch
{
private:
    /* data */
public:
    RRTGroupSearch(/* args */);

    virtual void updateMap(nav_msgs::OccupancyGrid map_data){
        std::cout << " this is updateMap function in base " << std::endl;
    }

    virtual void search(geometry_msgs::Point p_start, geometry_msgs::Point p_end){
        std::cout << " this is search function in base " << std::endl;
    }

    virtual bool getSearchState(){
        ;
    }

    virtual void getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                                  std::queue<geometry_msgs::Point> &p_final_q,
                                  int &iter){
        std::cout << " this is getVisInfo function in base " << std::endl;
        }

    ~RRTGroupSearch();

public:

    geometry_msgs::Point Steer(geometry_msgs::Point nearest_point, geometry_msgs::Point rand_point, float eta);
    geometry_msgs::Point Near(PointPair tree, geometry_msgs::Point p);
    geometry_msgs::Point Near(NodeCostPair tree, geometry_msgs::Point p);
    float Norm(geometry_msgs::Point p1, geometry_msgs::Point p2);
    double getRandData(int min,int max);
    bool nearGoal(geometry_msgs::Point new_point, geometry_msgs::Point end_point);
    int CollisionFree(geometry_msgs::Point nearest_point, geometry_msgs::Point new_point, nav_msgs::OccupancyGrid map);

};

RRTGroupSearch::RRTGroupSearch(/* args */)
{
    std::cout << "this is rrt group method " << std::endl;
}

RRTGroupSearch::~RRTGroupSearch()
{
}

// 返回 min max之间的随机数
double RRTGroupSearch::getRandData(int min,int max){
    double m1=(double)(rand()%101)/101; // 计算 0，1之间的随机小数,得到的值域近似为(0,1)
    min++;  //将 区间变为(min+1,max),
    double m2=(double)((rand()%(max-min+1))+min); //计算 min+1,max 之间的随机整数，得到的值域为[min+1,max]
    m2=m2-1; //令值域为[min,max-1]
    return m1+m2;  //返回值域为(min,max),为所求随机浮点数
}


// 计算rrt_tree上离随机点最近的点
geometry_msgs::Point RRTGroupSearch::Near(PointPair tree, geometry_msgs::Point p){
    geometry_msgs::Point nearst_point;
    float temp;
    float min = 100000;

    for (auto pt : tree)
    {
        temp = Norm(pt.first, p);
        if (temp < min)
        {
            min = temp;
            nearst_point = pt.first;
        }
    }

    return nearst_point;
}

geometry_msgs::Point RRTGroupSearch::Near(NodeCostPair tree, geometry_msgs::Point p){
    geometry_msgs::Point nearst_point;
    float temp;
    float min = 100000;

    for (auto node : tree)
    {
        temp = Norm(node.first, p);
        if (temp < min)
        {
            min = temp;
            nearst_point = node.first;
        }
    }

    return nearst_point;
    
}

//　返回两点的欧式距离
float RRTGroupSearch::Norm(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return pow( pow(p1.x-p2.x, 2) + pow(p1.y-p2.y,2), 0.5);
}

// 以nearest_point为起点，以nearest_point->rand_point射线为方向，前进eta的距离
geometry_msgs::Point RRTGroupSearch::Steer(geometry_msgs::Point nearest_point, geometry_msgs::Point rand_point, float eta){
    geometry_msgs::Point new_point;
    float dis = Norm(nearest_point, rand_point);
    if (dis <= eta)
    {
        new_point = rand_point;
    }else
    {
        new_point.x = nearest_point.x + eta * (rand_point.x - nearest_point.x) / dis;
        new_point.y = nearest_point.y + eta * (rand_point.y - nearest_point.y) / dis; 
    }
    return new_point;
}

bool RRTGroupSearch::nearGoal(geometry_msgs::Point new_point, geometry_msgs::Point end_point){
    if(Norm(new_point, end_point) < 0.5)
    {
        return true;
    }else
    {
        return false;
    }
}

// 占据状态　1:占据　0:未占据　-1:未知
int RRTGroupSearch::CollisionFree(geometry_msgs::Point nearest_point, geometry_msgs::Point new_point, nav_msgs::OccupancyGrid map){
    std::vector<signed char> Data = map.data;
    float re = map.info.resolution;
    float Pstartx = map.info.origin.position.x;
    float Pstarty = map.info.origin.position.y;
    float width = map.info.width;

    float step_length = map.info.resolution * 0.5;
    // //ceil(x)返回的是大于x的最小整数
    int step_number = ceil(Norm(nearest_point, new_point)/step_length);
    geometry_msgs::Point step_point = nearest_point;

    int state, out;
    for (int i = 0; i < step_number; i++)
    {
        step_point = Steer(step_point, new_point, step_length);
        // floor(x)返回的是小于或等于x的最大整数
        float index = floor((step_point.y - Pstarty)/re)*width + floor((step_point.x - Pstartx)/re);
        out = Data[int(index)];

        if (out == 100) // 被占据
        {
            state = 1;
            break;
        }

        if (out == -1) //未知
        {
            state = -1;
            break;
        }

        if (out == 0)
        {
            state = 0;
            continue;
        }
    }
    
    return state;

}
