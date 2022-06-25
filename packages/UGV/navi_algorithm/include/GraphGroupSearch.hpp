#pragma once
/**************************************************************************
 * GraphGroupSearch.hpp
 * 
 * @Author: bornchow
 * @Date: 2022.06.16
 * 
 * @Description:
 *  这里是图搜索的基类
 *  基类定义一些公用的函数和数据结构
 *  ****************************************************/
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

// #define DEBUG true

using namespace std;

// 定义节点
typedef struct Node
{
    int x;
    int y;
    float g;
    float h;
    float f;
    Node* father_node;
    vector<Node*> forcing_neis;
    Node(){

    }
    Node(int x, int y){
        this->x = x;
        this->y = y;
        this->g = 0.0;
        this->h = 0.0;
        this->f = 0.0;
        this->father_node = NULL;
        this->forcing_neis = {}; // for JPS
    }
    Node(int x, int y, Node* father){
        this->x = x;
        this->y = y;
        this->g = 0.0;
        this->h = 0.0;
        this->f = 0.0;
        this->father_node = father;
        this->forcing_neis = {};
    }

    // //重载 ==
    // bool operator == (const Node & n) const{
    //     if (x == n.x && y == n.y){
    //         return true;
    //     }else
    //     {
    //         return false;
    //     }
    // }
    // // 重载 < 
    // bool operator < (const Node &n) const {
    //     if (f != n.f)
    //     {
    //         return f < n.f;
    //     }else 
    //     {
    //         return h < n.h;
    //     }

    // }

    // 重载输出函数
    friend ostream& operator << (ostream& ostr, Node &n){

        if (n.father_node == NULL)
        {
            cout << "[ " << n.x << " , " << n.y << " ] -- " << "GHF: " << n.g << " " << n.h << " " << n.f 
            << " -- {  NULL  } " << std::endl;
        }else
        {
            cout << "[ " << n.x << " , " << n.y << " ] -- " << "GHF: " << n.g << " " << n.h << " " << n.f 
            << " -- { " << (n.father_node)->x<< " , " << (n.father_node)->y << " } ";
        }

        return cout
        ;
    } 

}Node;

// // 重载()运算符
// class NodeCompare
// {
//     public:
//         bool operator()(Node n1, Node n2){
//             if (n1.f < n2.f)
//             {
//                 return n1.f < n2.f;
//             }else if (n1.f = n2.f)
//             {
//                 return n1.h < n2.h;
//             }

//         }
// };

// ----------------------------------------------

//基类定义
class GraphGroupSearch
{
private:
    /* data */
public:
    GraphGroupSearch(/* args */);

    virtual void updateMap(nav_msgs::OccupancyGrid map_data){
        std::cout << " this is updateMap function in base " << std::endl;
        return;
    }

    virtual bool getSearchState(){
        return false;
    }

    virtual void search(geometry_msgs::Point p_start, geometry_msgs::Point p_end){
        std::cout << " this is search function in base " << std::endl;
        return;
    }

    virtual void getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                            std::queue<geometry_msgs::Point> &p_final_q,
                            int iter){
        std::cout << " this is getVisInfo function in base " << std::endl;
        return;
        }

    ~GraphGroupSearch();
public:
    void realCoordi2map(geometry_msgs::Point p_real, geometry_msgs::Point &p_map, nav_msgs::OccupancyGrid map_data);
    void map2realCoordi(geometry_msgs::Point &p_real, geometry_msgs::Point p_map, nav_msgs::OccupancyGrid map_data);
};

GraphGroupSearch::GraphGroupSearch(/* args */)
{
}

GraphGroupSearch::~GraphGroupSearch()
{
}

// **************************************
// 将点的真实坐标转换为地图坐标
// 这里的地图坐标是 A_star搜索中构建的地图，其以地图左下角(map_data.info.origin.position)为坐标原点， 地图的宽(map_data.info.width)为x方向 ，
//                 高(map_data.info.height)为y方向， 地图的分辨率(map_data.info.resolution)为大小构建的方格地图
//  ^ y
//  |
//  |
//  -----------> xstd::cout << "in close set " << std::endl;
void GraphGroupSearch::realCoordi2map(geometry_msgs::Point p_real, geometry_msgs::Point & p_map, nav_msgs::OccupancyGrid map_data){
    p_map.x = floor((p_real.x - map_data.info.origin.position.x)/map_data.info.resolution);
    p_map.y = floor((p_real.y - map_data.info.origin.position.y)/map_data.info.resolution);

}

// **************************************
// 将地图坐标点转换为真实坐标
void GraphGroupSearch::map2realCoordi(geometry_msgs::Point &p_real, geometry_msgs::Point p_map, nav_msgs::OccupancyGrid map_data){
    p_real.x = p_map.x * map_data.info.resolution + map_data.info.origin.position.x + 0.5 * map_data.info.resolution;
    p_real.y = p_map.y * map_data.info.resolution + map_data.info.origin.position.y + 0.5 * map_data.info.resolution;
}
