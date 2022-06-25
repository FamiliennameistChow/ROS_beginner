/**************************************************************************
 * DijkstraSearch.hpp
 * 
 * @Author： bornchow
 * @Date: 2022.06.16
 * 
 * @Description:
 *  本程序为Dijkstra路径搜索算法实现
 *  
 *  ****************************************************/

#include "ros/ros.h"
#include <iostream>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "tic_toc.h"
#include <mutex>
#include <map>
#include <set> 
#include <deque>
#include <queue>
#include <vector>
#include <algorithm>

#include "GraphGroupSearch.hpp"



// f(n) = g(n)

//这里使用multi_map来构建set
typedef std::multimap<float, Node*> NodeSets;
typedef std::multimap<float, Node*>::iterator NodeSetsIter;
// ----------------------------------

class DijkstraSearch : public GraphGroupSearch
{
private:
    int iter_;
    int iter_step_;
    bool map_updated_;
    bool search_finished_;

    nav_msgs::OccupancyGrid map_;

    NodeSets open_set_;
    NodeSets close_set_;

    // for vis
    std::mutex vis_mut_;
    std::map<int, std::queue<geometry_msgs::Point> > iter_vis_set_;
    std::deque<geometry_msgs::Point> p_final_que_;

private:
    void backSetNode(Node* node);
    void checkNeighbor(Node* thisNode, std::vector<Node*>& neiNodes);
    bool checkNode(Node* node, 
                    NodeSets& openSet, 
                    NodeSets closeSet, 
                    nav_msgs::OccupancyGrid map);
    int CollisionFree(Node* node, nav_msgs::OccupancyGrid map);
public:
    DijkstraSearch(int iter_step_);
    void updateMap(nav_msgs::OccupancyGrid map_data);
    bool getSearchState(); 
    void search(geometry_msgs::Point p_start, geometry_msgs::Point p_end);
    void getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                            std::queue<geometry_msgs::Point> &p_final_q,
                            int iter);
    ~DijkstraSearch();
};

DijkstraSearch::DijkstraSearch(int iter_step) :
iter_step_(iter_step)
{
    iter_ = 0;
    map_updated_ = false;
    search_finished_ = false;
    ROS_INFO("\033[1;32m----> this is a [2D] [Dijkstra] search method!! .\033[0m");
}

DijkstraSearch::~DijkstraSearch()
{
}

void DijkstraSearch::updateMap(nav_msgs::OccupancyGrid map_data){
    map_updated_ = true;
    map_ = map_data;
}

bool DijkstraSearch::getSearchState(){
    return search_finished_;
}

//****************************************
//// 【返回】 std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info
//  iter_vis_info.first 为 算法迭代次数
//  iter_vis_info.second 为 需要显示的新加点, 
void DijkstraSearch::getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                             std::queue<geometry_msgs::Point> &p_final_q,
                             int iter){
    
    vis_mut_.lock();
    if (!iter_vis_set_.empty())
    {
        copy(iter_vis_set_.begin(), iter_vis_set_.end(), inserter(iter_vis_info, iter_vis_info.begin()));
        iter_vis_set_.clear();
    }

    if (search_finished_)
    {
        while (!p_final_que_.empty())
        {
            p_final_q.push(p_final_que_.back()); //因为p_final_que_是从终点往起点储存的，所以这里要从后往前读，p_final_q才是从起点往终点储存
            p_final_que_.pop_back();
        }
    }

    vis_mut_.unlock();

    iter = iter_;
}

//**********************************
// 这里是算法主流程
//
//***********************************
void DijkstraSearch::search(geometry_msgs::Point p_start, geometry_msgs::Point p_end){
    
    while (!map_updated_)
    {
        ROS_INFO("no map info !! use updateMap function first");
    }
    
    //第一步 初始化open_set，将起始点加入open_set
    open_set_.clear();

    geometry_msgs::Point p_map_start, p_map_end;

    realCoordi2map(p_start, p_map_start, map_);
    Node* star_node = new Node(int(p_map_start.x), int(p_map_start.y));

    realCoordi2map(p_end, p_map_end, map_);
    Node* end_node = new Node(int(p_map_end.x), int(p_map_end.y));

    open_set_.insert(make_pair(star_node->g, star_node));

    // for vis
    std::queue<geometry_msgs::Point> vis_point_que;
    int a;

    //第二步 如果open_set为空， 返回false， 否则进入循环
    while (!open_set_.empty())
    {
        //为了让多线程轮转 ??
        std::this_thread::sleep_for(std::chrono::microseconds (1));

        while(!vis_point_que.empty()){ //清空显示
             vis_point_que.pop();
        }

        iter_++;

        if(iter_ % iter_step_ == 0){
                std::cout << "input any to go on!!" << std::endl;
                a = cin.get();
            }
        
        std::cout << "************************** " << iter_ << std::endl;

        // 第三步 从open_set中弹出一个$f(n) = g(n)$最小的节点，作为当前节点 Cur
        Node* current_node = open_set_.begin()->second; 

        #ifdef DEBUG
        std::cout << "cu node: " << *current_node << std::endl;
        #endif

        // 第四步 如果n是目标节点，表示搜索成功，回溯路径，否则进行下一步
        if(current_node->x == end_node->x && current_node->y == end_node->y){
            ROS_INFO("find the path!!!");
            search_finished_ = true;
            vis_mut_.lock();
            backSetNode(current_node);
            vis_mut_.unlock();
            ROS_INFO("back path over!!!");
            break;
        }

        std::cout << " --- " << open_set_.size() << std::endl;

        // 第五步 将当前节点从open_set中删除，然后加入close_set（标记当前节点为已扩展）
        auto it = open_set_.equal_range(current_node->g);
        std::cout << it.first->first << " " << it.second->first << std::endl;
        for(auto iter = it.first; iter != it.second; iter++){
            if(current_node->x == iter->second->x && current_node->y == iter->second->y){
                open_set_.erase(iter);
                break;
            }
        }

        std::cout << " --- " << open_set_.size() << std::endl;
        
        close_set_.insert(make_pair(current_node->g, current_node));

        //第六步 找到n的所有邻近节点
        std::vector<Node*> neighborNodes;
        checkNeighbor(current_node, neighborNodes);
        #ifdef DEBUG
        std::cout << " neighborNodes size: " << neighborNodes.size() << std::endl;
        #endif

        // 对每一个邻近节点处理
        geometry_msgs::Point p_new, p_map_new;
        for(int i=0; i<neighborNodes.size(); i++){
            neighborNodes[i]->father_node = current_node;
            if(checkNode(neighborNodes[i], open_set_, close_set_, map_)){
                //加入新节点以显示
                p_map_new.x = neighborNodes[i]->x;
                p_map_new.y = neighborNodes[i]->y;
                map2realCoordi(p_new, p_map_new, map_);
                vis_point_que.push(p_new);
            }
        }


        // for vis
        vis_mut_.lock();
        iter_vis_set_.insert(make_pair(iter_, vis_point_que));
        vis_mut_.unlock();

        //循环


    }
    
}


//*******************************************
// 这里是检查邻近节点node是否加入open_set_ 
// 对应算法中的第六步下面的步骤
bool DijkstraSearch::checkNode(Node* node, 
                                NodeSets& openSet, 
                                NodeSets closeSet, 
                                nav_msgs::OccupancyGrid map){
    
    std::cout << "--[check node] --------[ " << node->x << " , " << node->y <<" ]" << std::endl;
    //检测node是否在地图范围
    if(node->x < 0 || node->x > int(map.info.width) || node->y < 0 || node->y > int(map.info.height)){
        #ifdef DEBUG
        std::cout << "--[check node] outside map " << std::endl;
        #endif
        return false;
    }

    // -->判断其地图的占据状况，如果占据，丢弃它
    int checking = CollisionFree(node, map);
    switch (checking)
    {
    case 1: 
    {
        #ifdef DEBUG
        std::cout << "--[check node] collision 1 " << std::endl;
        #endif
        return false;
    }
    case -1:
    {
        #ifdef DEBUG
        std::cout << "--[check node] collision -1 " << std::endl;
        #endif
        return false;
    }
    case 0: //free状态
    {
        #ifdef DEBUG
        std::cout << "--[check node] collision free " << std::endl;
        #endif

        // --> 判断是否在close_set中，如果是，丢弃它
        for(NodeSetsIter iter=closeSet.begin(); iter != closeSet.end(); iter++){
            if(node->x == iter->second->x && node->y == iter->second->y){
                #ifdef DEBUG
                std::cout << "--[check node] in close set " << std::endl;
                #endif
                return false;
            }
        }

        // -->计算代价
        node->g = node->father_node->g + sqrt(pow(node->x - node->father_node->x, 2) + 
                                            pow(node->y - node->father_node->y, 2));
        
        // --> 判断是否在 open_set中

        for(NodeSetsIter iter = openSet.begin(); iter != openSet.end(); iter++){
            if(node->x == iter->second->x && node->y == iter->second->y){
                // 表示在openSet中
                #ifdef DEBUG
                std::cout << "--[check node] in open set " << std::endl;
                #endif
                if(iter->first > node->g){
                    openSet.erase(iter);
                    openSet.insert(make_pair(node->g, node));
                    //这里node替换了iter
                }
                return false;
            }
        }

        // 表示不在openset中
        std::cout << "--[check node] add new node  " << std::endl;
        openSet.insert(make_pair(node->g, node));
        return true;
    }
    
    default:
        return false;
    }
}


//*******************************************
// 返回map坐标系中 node坐标的占有状态
// 占据状态　1:占据　0:未占据　-1:未知
int DijkstraSearch::CollisionFree(Node* node, nav_msgs::OccupancyGrid map){

    std::vector<signed char> Data = map.data;
    int index = node->y * map.info.width + node->x;

    int state, out;
    out = Data[index];

    if (out == 100) // 被占据
    {
        state = 1;
    }

    if (out == -1) //未知
    {
        state = -1;
    }

    if (out == 0)
    {
        state = 0;
    }
    
    return state;

}


//*******************************
// 搜索当前节点的临近节点
// 这里采用八连通的搜索方式，　既是一个节点可以衍生出八个临近节点
//【输入】thisNode 当前节点
//【输出】neiNodes 当前节点的临近节点 
void DijkstraSearch::checkNeighbor(Node* thisNode, std::vector<Node*>& neiNodes){
    neiNodes.push_back(new Node(thisNode->x,   thisNode->y+1));
    neiNodes.push_back(new Node(thisNode->x,   thisNode->y-1));
    neiNodes.push_back(new Node(thisNode->x-1, thisNode->y));
    neiNodes.push_back(new Node(thisNode->x-1, thisNode->y+1));
    neiNodes.push_back(new Node(thisNode->x-1, thisNode->y-1));
    neiNodes.push_back(new Node(thisNode->x+1, thisNode->y));
    neiNodes.push_back(new Node(thisNode->x+1, thisNode->y+1));
    neiNodes.push_back(new Node(thisNode->x+1, thisNode->y-1));

}

void DijkstraSearch::backSetNode(Node* node){
    if(node == nullptr) return;

    geometry_msgs::Point p, p_map;
    p_map.x = node->x;
    p_map.y = node->y;
    map2realCoordi(p, p_map, map_);
    p_final_que_.push_back(p);

    backSetNode(node->father_node);

}


