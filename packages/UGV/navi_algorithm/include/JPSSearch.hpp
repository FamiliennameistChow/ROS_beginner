/**************************************************************************
 * JPSSearch.hpp
 * 
 * @Author： bornchow
 * @Date: 2022.06.22
 * 
 * @Description:
 *  本程序为JPS路径搜索算法实现
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
#include <thread>

#include "GraphGroupSearch.hpp"



enum Dir{
    left, //0
    up,  //1
    right,  //2
    down, // 3
    leftUp,
    rightUp,
    leftDown,
    rightDwon,  
};

typedef std::multimap<float, Node*> JPSNodeSets;
typedef std::multimap<float, Node*>::iterator JPSNodeSetsIter;

class JPSSearch : public GraphGroupSearch
{
private:
    int iter_;
    int heuristic_func_type_;
    int iter_step_;
    bool map_updated_;
    bool search_finished_;

    nav_msgs::OccupancyGrid map_;

    // for vis
    std::mutex vis_mut_;
    std::map<int, std::queue<geometry_msgs::Point> > iter_vis_set_;
    std::deque<geometry_msgs::Point> p_final_que_;

    JPSNodeSets open_set_, close_set_;
private:
    void backSetNode(Node* node);
    void searchJumpPoint(JPSNodeSets& openSet,
                        JPSNodeSets closeSet,
                        Node *curNode, 
                        Node *endNode,
                        std::queue<geometry_msgs::Point>& vis_JP, 
                        nav_msgs::OccupancyGrid map);
    void nodeToDirs(Node* a, Node* b, std::queue<int>& dirList);
    int dirSearchState(Node* node, nav_msgs::OccupancyGrid map, int dir, Node* endNode);
    void addNodeToOpenSet(JPSNodeSets& openSet, JPSNodeSets closeSet, Node* node, Node *endNode);
    int CollisionFree(Node* node, nav_msgs::OccupancyGrid map);
    bool isJumpPoint(Node* node, nav_msgs::OccupancyGrid map, int dir);
    void countCost(Node* node, Node* end_node);
public:
    JPSSearch(int heuristic_func_type, int iter_step);
    void updateMap(nav_msgs::OccupancyGrid map_data);
    void search(geometry_msgs::Point p_start, geometry_msgs::Point p_end);
    bool getSearchState(); 
    void getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                        std::queue<geometry_msgs::Point> &p_final_q,
                        int iter);
    ~JPSSearch();
};

JPSSearch::JPSSearch(int heuristic_func_type, int iter_step):
iter_step_(iter_step),
heuristic_func_type_(heuristic_func_type)
{
    iter_ = 0;
    map_updated_ = false;
    search_finished_ = false;
    ROS_INFO("\033[1;32m----> this is a [2D] [JPS] search method!! .\033[0m");
}

JPSSearch::~JPSSearch()
{
}

bool JPSSearch::getSearchState(){
    return search_finished_;
}

void JPSSearch::updateMap(nav_msgs::OccupancyGrid map_data){
    map_updated_ = true;
    map_ = map_data;
}

//****************************************
//// 【返回】 std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info
//  iter_vis_info.first 为 算法迭代次数
//  iter_vis_info.second 为 需要显示的新加点, 
void JPSSearch::getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
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

void JPSSearch::search(geometry_msgs::Point p_start, geometry_msgs::Point p_end){

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

    open_set_.insert(make_pair(star_node->f, star_node));

    // for vis
    std::queue<geometry_msgs::Point> vis_point_que; //这里只记录跳点
    int a;

    //第二步 如果open_set为空， 返回false， 否则进入循环
    while(!open_set_.empty()){
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

        // 第五步 将当前节点从open_set中删除，然后加入close_set（标记当前节点为已扩展）
        auto it = open_set_.equal_range(current_node->f);
        std::cout << it.first->first << " " << it.second->first << std::endl;
        for(auto iter = it.first; iter != it.second; iter++){
            if(current_node->x == iter->second->x && current_node->y == iter->second->y){
                open_set_.erase(iter);
                break;
            }
        }

        std::cout << " --- open_set size: " << open_set_.size() << std::endl;
        
        close_set_.insert(make_pair(current_node->f, current_node));

        // 前五步与dijkstra算法一致

        // 第六步 跳点搜索
        searchJumpPoint(open_set_, close_set_, current_node, end_node, vis_point_que, map_);

        // for vis
        vis_mut_.lock();
        iter_vis_set_.insert(make_pair(iter_, vis_point_que));
        vis_mut_.unlock();

        std::cout << " --- open_set size:" << open_set_.size() << std::endl;

        // if(iter_)


    }

}

// *******************************
// 搜索跳点
// 这里是JPS不同于 A*的地方
void JPSSearch::searchJumpPoint(JPSNodeSets& openSet, 
                                JPSNodeSets closeSet, 
                                Node *thisNode, 
                                Node *endNode,
                                std::queue<geometry_msgs::Point>& vis_JP, 
                                nav_msgs::OccupancyGrid map){
    // 定义当前节点的搜索方向序列
    std::queue<int> dirList; //添加方向的时候, 是先按直线方向，再按斜线方向
    if (thisNode->father_node == nullptr) //如果当前节点没有父节点,(起点)，那么搜索方向是八个方向
    {
        dirList.push(Dir::left);
        dirList.push(Dir::up);
        dirList.push(Dir::right);
        dirList.push(Dir::down);
        dirList.push(Dir::leftUp);
        dirList.push(Dir::rightUp);
        dirList.push(Dir::leftDown);
        dirList.push(Dir::rightDwon);
    }else //如果当前节点有父节点
    {   
        // father --> thisNode
        nodeToDirs(thisNode->father_node, thisNode, dirList);
    }
    
    if (!thisNode->forcing_neis.empty()) //如果当前节点有强迫邻居
    {
        for(int i=0; i<thisNode->forcing_neis.size(); i++){
            // thisNode --> forcing_node
            nodeToDirs(thisNode, thisNode->forcing_neis[i], dirList);
        }
    }

    std::cout << " dirs size: " << dirList.size() << std::endl;

    //按方向进行搜索
    int searchState = 0;
    geometry_msgs::Point p_new, p_map_new;
    while (!dirList.empty())
    {
        int thisDir = dirList.front();
        dirList.pop();

        switch (thisDir)
        {
        case Dir::left:{ //(-1, 0)
            #ifdef DEBUG
            std::cout << "-- search dir --> left" << std::endl;
            #endif
            Node* checkNode = new Node(thisNode->x - 1, thisNode->y, thisNode);
            searchState = dirSearchState(checkNode, map, thisDir, endNode);
            while ( searchState == 0)
            {
                checkNode->x -= 1;
                searchState = dirSearchState(checkNode, map, thisDir, endNode);
            }

            if(searchState == 3){ //checkNode是跳点
                addNodeToOpenSet(openSet, closeSet, checkNode, endNode);

                //for vis
                p_map_new.x = checkNode->x;
                p_map_new.y = checkNode->y;
                map2realCoordi(p_new, p_map_new, map);
                vis_JP.push(p_new);
                //for vis
            }
            
    
            break;
        }

        case Dir::right:{
            #ifdef DEBUG
            std::cout << "-- search dir --> right" << std::endl;
            #endif
            Node* checkNode = new Node(thisNode->x + 1, thisNode->y, thisNode);
            searchState = dirSearchState(checkNode, map, thisDir, endNode);
            while (searchState == 0)
            {
                checkNode->x += 1;
                searchState = dirSearchState(checkNode, map, thisDir, endNode);
            }

            if(searchState == 3){ //checkNode是跳点
                addNodeToOpenSet(openSet, closeSet, checkNode, endNode);

                //for vis
                p_map_new.x = checkNode->x;
                p_map_new.y = checkNode->y;
                map2realCoordi(p_new, p_map_new, map);
                vis_JP.push(p_new);
                //for vis
            }
            
          
            break;
        }

        case Dir::up:{
            #ifdef DEBUG
            std::cout << "-- search dir --> up" << std::endl;
            #endif
            Node* checkNode = new Node(thisNode->x, thisNode->y+1, thisNode);
            searchState = dirSearchState(checkNode, map, thisDir, endNode);
            while (searchState == 0)
            {
                checkNode->y += 1;
                searchState = dirSearchState(checkNode, map, thisDir, endNode);
            }

            if(searchState == 3){ //checkNode是跳点
                addNodeToOpenSet(openSet, closeSet, checkNode, endNode);

                //for vis
                p_map_new.x = checkNode->x;
                p_map_new.y = checkNode->y;
                map2realCoordi(p_new, p_map_new, map);
                vis_JP.push(p_new);
                //for vis
            }
            
           
            break;

        }

        case Dir::down:{
            #ifdef DEBUG
            std::cout << "-- search dir --> down" << std::endl;
            #endif
            Node* checkNode = new Node(thisNode->x, thisNode->y-1, thisNode);
            searchState = dirSearchState(checkNode, map, thisDir, endNode);
            while (searchState == 0)
            {
                checkNode->y -= 1;
                searchState = dirSearchState(checkNode, map, thisDir, endNode);
            }

            if(searchState == 3){ //checkNode是跳点
                addNodeToOpenSet(openSet, closeSet, checkNode, endNode);

                //for vis
                p_map_new.x = checkNode->x;
                p_map_new.y = checkNode->y;
                map2realCoordi(p_new, p_map_new, map);
                vis_JP.push(p_new);
                //for vis
            }
            
            
            break;
        }
        case Dir::leftUp:{ //这里是斜向搜索
            #ifdef DEBUG
            std::cout << "-- search dir --> leftUp" << std::endl;
            #endif
            bool hasJP = false;
            Node* checkNode = new Node(thisNode->x-1, thisNode->y+1, thisNode); // 向该方向前进一步,注意斜向节点的父节点被设置为了thisNode
            while (dirSearchState(checkNode, map, thisDir, endNode) == 0)
            {
                // 向上的分量， 向上搜索的节点的父节点被设置为了斜向节点
                Node* upNode = new Node(checkNode->x, checkNode->y, checkNode);
                searchState = dirSearchState(upNode, map, Dir::up, endNode);
                while (searchState == 0)
                {
                    upNode->y += 1;
                    searchState = dirSearchState(upNode, map, Dir::up, endNode);
                }
                if(searchState == 3){ //up方向上搜索到跳点，upNode是跳点，同时checkNode 也是跳点
                    // 将checkNode加入openSet --> 这一步会给checkNode赋f,g,h值， 
                    addNodeToOpenSet(openSet, closeSet, checkNode, endNode);

                    // 将upNode加入openSet
                    addNodeToOpenSet(openSet, closeSet, upNode, endNode);
                    hasJP = true;

                    //for vis
                    p_map_new.x = checkNode->x;
                    p_map_new.y = checkNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);

                    p_map_new.x = upNode->x;
                    p_map_new.y = upNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);
                    //for vis

                }

                // 向左的分量
                Node* leftNode = new Node(checkNode->x, checkNode->y, checkNode);
                searchState = dirSearchState(leftNode, map, Dir::left, endNode);
                while (searchState == 0)
                {
                    leftNode->x -= 1;
                    searchState = dirSearchState(leftNode, map, Dir::left, endNode);
                }
                if(searchState == 3){ //left方向上搜索到跳点，leftNode是跳点，同时checkNode 也是跳点
                    // 将checkNode加入openSet --> 这一步会给checkNode赋f,g,h值， 
                    addNodeToOpenSet(openSet, closeSet, checkNode, endNode);

                    // 将leftNode加入openSet
                    addNodeToOpenSet(openSet, closeSet, leftNode, endNode);
                    hasJP = true;

                    // for vis
                    p_map_new.x = checkNode->x;
                    p_map_new.y = checkNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);

                    p_map_new.x = leftNode->x;
                    p_map_new.y = leftNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);

                }

                if(hasJP){

                    break;
                }

                // 向leftUp 前进一步
                checkNode->x -= 1;
                checkNode->y += 1;
            }

            break;
        }

        case Dir::leftDown:{
            #ifdef DEBUG
            std::cout << "-- search dir --> leftDown" << std::endl;
            #endif
            bool hasJP = false;
            Node* checkNode = new Node(thisNode->x-1, thisNode->y-1, thisNode); // 向该方向前进一步
            while (dirSearchState(checkNode, map, thisDir, endNode) == 0)
            {
                // 向下的分量
                Node* downNode = new Node(checkNode->x, checkNode->y, checkNode);
                searchState = dirSearchState(downNode, map, Dir::down, endNode);
                while (searchState == 0)
                {
                    downNode->y -= 1;
                    searchState = dirSearchState(downNode, map, Dir::down, endNode);
                }
                if(searchState == 3){ //up方向上搜索到跳点，downNode是跳点，同时checkNode 也是跳点
                    // 将checkNode加入openSet --> 这一步会给checkNode赋f,g,h值， 
                    addNodeToOpenSet(openSet, closeSet, checkNode, endNode);

                    // 将downNode加入openSet
                    addNodeToOpenSet(openSet, closeSet, downNode, endNode);
                    hasJP = true;

                    // for vis
                    p_map_new.x = checkNode->x;
                    p_map_new.y = checkNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);

                    p_map_new.x = downNode->x;
                    p_map_new.y = downNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);
                }

                // 向左的分量
                Node* leftNode = new Node(checkNode->x, checkNode->y, checkNode);
                searchState = dirSearchState(leftNode, map, Dir::left, endNode);
                while (searchState == 0)
                {
                    leftNode->x -= 1;
                    searchState = dirSearchState(leftNode, map, Dir::left, endNode);
                }
                if(searchState == 3){ //left方向上搜索到跳点，leftNode是跳点，同时checkNode 也是跳点
                    // 将checkNode加入openSet --> 这一步会给checkNode赋f,g,h值， 
                    addNodeToOpenSet(openSet, closeSet, checkNode, endNode);

                    // 将leftNode加入openSet
                    addNodeToOpenSet(openSet, closeSet, leftNode, endNode);
                    hasJP = true;

                    // for vis
                    p_map_new.x = checkNode->x;
                    p_map_new.y = checkNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);

                    p_map_new.x = leftNode->x;
                    p_map_new.y = leftNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);
                }

                if(hasJP){
                    break;
                }

                // 向leftDown 前进一步
                checkNode->x -= 1;
                checkNode->y -= 1;
            }

            break;
        }

        case Dir::rightDwon:{
            #ifdef DEBUG
            std::cout << "-- search dir --> rightDwon" << std::endl;
            #endif
            bool hasJP = false;
            Node* checkNode = new Node(thisNode->x+1, thisNode->y-1, thisNode); // 向该方向前进一步
            while (dirSearchState(checkNode, map, thisDir, endNode) == 0)
            {
                // 向下的分量
                Node* downNode = new Node(checkNode->x, checkNode->y, checkNode);
                searchState = dirSearchState(downNode, map, Dir::down, endNode);
                while (searchState == 0)
                {
                    downNode->y -= 1;
                    searchState = dirSearchState(downNode, map, Dir::down, endNode);
                }
                if(searchState == 3){ //up方向上搜索到跳点，downNode是跳点，同时checkNode 也是跳点
                    // 将checkNode加入openSet --> 这一步会给checkNode赋f,g,h值， 
                    addNodeToOpenSet(openSet, closeSet, checkNode, endNode);

                    // 将downNode加入openSet
                    addNodeToOpenSet(openSet, closeSet, downNode, endNode);
                    hasJP = true;

                    // for vis
                    p_map_new.x = checkNode->x;
                    p_map_new.y = checkNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);

                    p_map_new.x = downNode->x;
                    p_map_new.y = downNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);
                }

                // 向右的分量
                Node* rightNode = new Node(checkNode->x, checkNode->y, checkNode);
                searchState = dirSearchState(rightNode, map, Dir::right, endNode);
                while (searchState == 0)
                {
                    rightNode->x += 1;
                    searchState = dirSearchState(rightNode, map, Dir::right, endNode);
                }
                if(searchState == 3){ //left方向上搜索到跳点，rightNode是跳点，同时checkNode 也是跳点
                    // 将checkNode加入openSet --> 这一步会给checkNode赋f,g,h值， 
                    addNodeToOpenSet(openSet, closeSet, checkNode, endNode);

                    // 将rightNode加入openSet
                    addNodeToOpenSet(openSet, closeSet, rightNode, endNode);
                    hasJP = true;

                    // for vis
                    p_map_new.x = checkNode->x;
                    p_map_new.y = checkNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);

                    p_map_new.x = rightNode->x;
                    p_map_new.y = rightNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);
                }

                if(hasJP){
                    break;
                }

                // 向rightDwon 前进一步
                checkNode->x += 1;
                checkNode->y -= 1;
            }

            break;
        }
        case Dir::rightUp:{
            #ifdef DEBUG
            std::cout << "-- search dir --> rightUp" << std::endl;
            #endif
            bool hasJP = false;
            Node* checkNode = new Node(thisNode->x+1, thisNode->y+1, thisNode); // 向该方向前进一步
            while (dirSearchState(checkNode, map, thisDir, endNode) == 0)
            {
                // 向上的分量
                Node* upNode = new Node(checkNode->x, checkNode->y, checkNode);
                searchState = dirSearchState(upNode, map, Dir::up, endNode);
                while (searchState == 0)
                {
                    upNode->y += 1;
                    searchState = dirSearchState(upNode, map, Dir::up, endNode);
                }
                if(searchState == 3){ //up方向上搜索到跳点，upNode是跳点，同时checkNode 也是跳点
                    // 将checkNode加入openSet --> 这一步会给checkNode赋f,g,h值， 
                    addNodeToOpenSet(openSet, closeSet, checkNode, endNode);

                    // 将upNode加入openSet
                    addNodeToOpenSet(openSet, closeSet, upNode, endNode);
                    hasJP = true;

                    // for vis
                    p_map_new.x = checkNode->x;
                    p_map_new.y = checkNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);

                    p_map_new.x = upNode->x;
                    p_map_new.y = upNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);
                }

                // 向右的分量
                Node* rightNode = new Node(checkNode->x, checkNode->y, checkNode);
                searchState = dirSearchState(rightNode, map, Dir::right, endNode);
                while (searchState == 0)
                {
                    rightNode->x += 1;
                    searchState = dirSearchState(rightNode, map, Dir::right, endNode);
                }
                if(searchState == 3){ //left方向上搜索到跳点，rightNode是跳点，同时checkNode 也是跳点
                    // 将checkNode加入openSet --> 这一步会给checkNode赋f,g,h值， 
                    addNodeToOpenSet(openSet, closeSet, checkNode, endNode);

                    // 将rightNode加入openSet
                    addNodeToOpenSet(openSet, closeSet, rightNode, endNode);
                    hasJP = true;

                    // for vis
                    p_map_new.x = checkNode->x;
                    p_map_new.y = checkNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);

                    p_map_new.x = rightNode->x;
                    p_map_new.y = rightNode->y;
                    map2realCoordi(p_new, p_map_new, map);
                    vis_JP.push(p_new);
                }

                if(hasJP){
                    break;
                }

                // 向rightUp 前进一步
                checkNode->x += 1;
                checkNode->y += 1;
            }

            break;
        }

        default:
            break;
        }
    }
    
    
}

// ***********************************
// 判断该方向的搜索状态
// 返回值
// 1. 搜索结束 当前节点超出边界
// 2. 搜索结束 当前节点是障碍物
// 3. 搜索结束 当前节点是跳点 
// 0. 该方向可继续搜索

int JPSSearch::dirSearchState(Node* node, nav_msgs::OccupancyGrid map, int dir, Node* endNode){
    // 1. 判断节点是否超过边界
    if(node->x < 0 || node->x > map.info.width || node->y <0 || node->y > map.info.height){
        #ifdef DEBUG
        std::cout << "[dirSearchState] check over by out of map" << std::endl;
        #endif
        return 1;
    }

    // 判断节点是否是障碍
    int checking = CollisionFree(node, map);
    if(checking == 1 || checking == -1){
        #ifdef DEBUG
        std::cout << "[dirSearchState] check over by hit ob" << std::endl;
        #endif
        return 2;
    }

    
    
    if(dir <=3){  //如果是斜向运动，暂不判定跳点,交由斜向运动的直线分量判定

        //目标点也是跳点
        if(node->x == endNode->x && node->y == endNode->y){
            #ifdef DEBUG
            std::cout << "[dirSearchState] check over by hit target" << std::endl;
            #endif

            return 3;
        }

        //判断是否是跳点
        if(isJumpPoint(node, map, dir)){
            #ifdef DEBUG
            std::cout << "[dirSearchState] check over by JP" << std::endl;
            #endif

            return 3;
        }
    }

    return 0;
}

// 将跳点加入openSet
// 
// Node* node 给node赋了f,g,h
void JPSSearch::addNodeToOpenSet(JPSNodeSets& openSet, JPSNodeSets closeSet, Node* node, Node* endNode){
    // 1. 判断跳点是否在close_set中
    for(JPSNodeSetsIter iter=closeSet.begin(); iter != closeSet.end(); iter++){
        if(node->x == iter->second->x && node->y == iter->second->y){
            #ifdef DEBUG
            std::cout << "--[add node] in close set " << std::endl;
            #endif
            return;
        }
    }

    // 2. 计算跳点代价
    countCost(node, endNode);
    // 3. 判断跳点是否在open_set中
    for(JPSNodeSetsIter iter=openSet.begin(); iter != openSet.end(); iter++){
        if(node->x == iter->second->x && node->y == iter->second->y){
            #ifdef DEBUG
            std::cout << "--[add node] in open set " << std::endl;
            #endif
            if(node->f < iter->second->f){ //从这条路cost更小
                openSet.erase(iter);
                openSet.insert(make_pair(node->f, node));
            }
            return;
        }
    }

    // 不在open_set中
    #ifdef DEBUG
    std::cout << "--[add node] new node " << std::endl;
    #endif
    openSet.insert(make_pair(node->f, node));
}



//**********************************
// 计算当前节点的代价
//【输入】 node 当前节点  node 值中的 f, g, h会被修改
//【输入】 end_node 终点
// 调用全局变量 heuristic_func_type 启发函数类型， 用什么方法计算当前节点与终点的启发值
// 0 - Manhattan Distance 曼哈顿距离
// 1 - Diagonal distance 对角距离
// 2 - Euclidean distance 欧几里得距离(直线距离)
//【输出】 node 赋值f,g,h的当前节点值
// 参考： https://zhuanlan.zhihu.com/p/54510444
void JPSSearch::countCost(Node* node, Node* end_node){
    float f, g, h;
    float dx = abs(node->x - end_node->x);
    float dy = abs(node->y - end_node->y);

    // 计算 h
    if (heuristic_func_type_ == 0) 
    {
        h = dx + dy;
    }else if (heuristic_func_type_ == 1)
    {
        h = dx + dy + (sqrt(2) -2 ) * min(dx, dy);
    }else if (heuristic_func_type_ == 2)
    {
        h = pow((dx, 2) + pow(dy, 2), 0.5);
    }
    
    // 计算 g = father.g + node 到 father_node的距离
    g = node->father_node->g + sqrt(pow(node->x - node->father_node->x,2) + pow(node->y - node->father_node->y,2));

    f = g + h;

    node->f = f;
    node->g = g;
    node->h = h;

}

// 确定 a->b的方向 序列
void JPSSearch::nodeToDirs(Node* a, Node* b, std::queue<int>& dirs){
    int deltaX = b->x - a->x;
    int deltaY = b->y - a->y;
    if(deltaX > 0 && deltaY > 0){ //(1,1) --> 方向为 rightUp
        dirs.push(Dir::right);
        dirs.push(Dir::up);
        dirs.push(Dir::rightUp);
    }else if(deltaX > 0 && deltaY == 0){ //(1, 0) --> 方向为 right
        dirs.push(Dir::right);
    }else if(deltaX > 0 && deltaY < 0){ //(1, -1) --> 方向为 rightDown 
        dirs.push(Dir::right);
        dirs.push(Dir::down);
        dirs.push(Dir::rightDwon);
    }else if (deltaX == 0 && deltaY > 0){ //(0, 1) --> 方向为 Up
        dirs.push(Dir::up);
    }else if(deltaX == 0 && deltaY < 0){ //(0, -1) --> 方向为 down
        dirs.push(Dir::down);
    }else if(deltaX < 0 && deltaY > 0){ //(-1, 1) --> 方向为 leftUp
        dirs.push(Dir::left);
        dirs.push(Dir::up);
        dirs.push(Dir::leftUp);
    }else if(deltaX < 0 && deltaY == 0){// (-1, 0) --> 方向为left
        dirs.push(Dir::left);
    }else if(deltaX < 0 && deltaY < 0){ //(-1, -1) --> 方向为leftDown
        dirs.push(Dir::left);
        dirs.push(Dir::down);
        dirs.push(Dir::leftDown);
    }else{
        ROS_ERROR("error deltaX and deltaY value");
    }  
}

void JPSSearch::backSetNode(Node* node){
    if(node == nullptr) return;

    geometry_msgs::Point p, p_map;
    p_map.x = node->x;
    p_map.y = node->y;
    map2realCoordi(p, p_map, map_);
    p_final_que_.push_back(p);

    backSetNode(node->father_node);

}

//*******************************************
// 返回map坐标系中 node坐标的占有状态
// 占据状态　1:占据　0:未占据　-1:未知
int JPSSearch::CollisionFree(Node* node, nav_msgs::OccupancyGrid map){

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

//*******************************************
// 检查该节点是不是跳点
//     * * *     upLeftIndex    upIndex      upRightIndex
//     * x *      leftIndex     thisIndex    rightIndex
//     * * *    downLeftIndex   downIndex   downRightIndex
// 该函数只能检测水平和竖直方向的跳点
bool JPSSearch::isJumpPoint(Node* node, nav_msgs::OccupancyGrid map, int dir){

    std::vector<signed char> Data = map.data;
    int thisIndex = node->y * map.info.width + node->x;

    int upIndex = (node->y + 1) * map.info.width + node->x;
    int downIndex = (node->y - 1) * map.info.width + node->x;
    int rightIndex = node->y * map.info.width + node->x +1;
    int leftIndex = node->y * map.info.width + node->x -1;
    int upLeftIndex = (node->y + 1) * map.info.width + node->x -1;
    int upRightIndex = (node->y + 1) * map.info.width + node->x +1;
    int downLeftIndex = (node->y - 1) * map.info.width + node->x -1;
    int downRightIndex = (node->y - 1) * map.info.width + node->x +1;

    bool isJp = false;

    switch (dir)
    {
    case Dir::left:{
        if(Data[leftIndex] != 0){
            return false;
        }

        if(Data[upIndex] != 0 && Data[upLeftIndex] == 0){ // node 是跳点 upLeftIndex是强迫邻居
            //将强迫邻居赋给node节点
            Node* neiNode = new Node(node->x-1, node->y+1);
            node->forcing_neis.push_back(neiNode);
            isJp = true;
        }

        if(Data[downIndex] !=0 && Data[downLeftIndex] == 0){ // node 是跳点 downLeftIndex是强迫邻居
            Node* neiNode = new Node(node->x-1, node->y-1);
            node->forcing_neis.push_back(neiNode);
            isJp = true;
        }

        break;
    }

    case Dir::right:{
        if(Data[rightIndex] != 0){
            return false;
        }

        if(Data[upIndex] != 0 && Data[upRightIndex] == 0){ // node 是跳点 upRightIndex是强迫邻居
            Node* neiNode = new Node(node->x+1, node->y+1);
            node->forcing_neis.push_back(neiNode);
            isJp = true;
        }

        if(Data[downIndex] != 0 && Data[downRightIndex] == 0){ // node 是跳点 downRightIndex是强迫邻居
            Node* neiNode = new Node(node->x+1, node->y-1);
            node->forcing_neis.push_back(neiNode);
            isJp = true;
        }
        break;
    }

    case Dir::up:{
        if(Data[upIndex] != 0){
            return false;
        }

        if(Data[leftIndex] != 0 && Data[upLeftIndex] == 0){  // node 是跳点 upLeftIndex是强迫邻居
            Node* neiNode = new Node(node->x-1, node->y+1);
            node->forcing_neis.push_back(neiNode);
            isJp = true;
        }

        if(Data[rightIndex] != 0 && Data[upRightIndex] == 0){ // node 是跳点 upRightIndex是强迫邻居
            Node* neiNode = new Node(node->x+1, node->y+1);
            node->forcing_neis.push_back(neiNode);
            isJp = true;
        }


        break;
    }

    case Dir::down:{
        if(Data[downIndex] != 0){
            return false;
        }

        if(Data[leftIndex] !=0 && Data[downLeftIndex] == 0){ // node 是跳点 downLeftIndex是强迫邻居
            Node* neiNode = new Node(node->x-1, node->y-1);
            node->forcing_neis.push_back(neiNode);
            isJp = true;
        }

        if(Data[rightIndex] != 0 && Data[downRightIndex] == 0){ // node 是跳点 downRightIndex是强迫邻居
            Node* neiNode = new Node(node->x+1, node->y-1);
            node->forcing_neis.push_back(neiNode);
            isJp = true;
        }
        break;
    }

    default:
        break;
    }
    
    if(isJp) return true;

    return false;

}

