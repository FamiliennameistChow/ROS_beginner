/**************************************************************************
 * RRTStarSearch.hpp
 * 
 * @Author： bornchow
 * @Date: 2022.03.01
 * 
 * @Description:
 *  本程序为rrt系列路径搜索算法中rrt-star的实现
 * 1. 实现rrt-star算法
 * 2. 将需要显示的信息反馈
 *  
 *  ****************************************************/
#include "RRTGroupSearch.hpp"
#include "tic_toc.h"
#include <mutex>

class RRTStarSearch : public RRTGroupSearch
{
private:
    NodeCostPair rrt_tree_set_;
    nav_msgs::OccupancyGrid map_;
    bool map_updated_;
    bool search_finished_;
    int iter_; // 计数迭代次数

    geometry_msgs::Point p_new_, p_rand_, p_nearest_;
    geometry_msgs::Point p_start_, p_end_;
    NodeCost new_node_;

    std::map<int, std::queue<geometry_msgs::Point> > iter_vis_set_;
    std::queue<geometry_msgs::Point> p_final_que_;
    std::mutex vis_mut_;

    // 参数
    float eta_;
    int iter_step_;
    int iter_max_;
    float search_radius_;

private:
    NodeCost NearSearch(geometry_msgs::Point p_new, geometry_msgs::Point p_nearest, 
                        float r, NodeCostPair tree, nav_msgs::OccupancyGrid map);
    void RewireTree( NodeCostPair &tree, geometry_msgs::Point p_new, float r, nav_msgs::OccupancyGrid map);

public:
    RRTStarSearch(float eta, int iter_step, int iter_max, float search_radius);
    virtual void updateMap(nav_msgs::OccupancyGrid map_data);
    virtual void search(geometry_msgs::Point p_start, geometry_msgs::Point p_end);
    virtual bool getSearchState();
    virtual void getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                                  std::queue<geometry_msgs::Point> &p_final_q,
                                  int &iter);
    ~RRTStarSearch();
};

//******************************************************
// 【输入】 - eta : 每隔迭代中rrt树扩展的距离 
// 【输入】 - iter_step : 每隔多少步，需要键入以继续
// 【输入】 - iter_max : 最大迭代次数
// 【输入】 - search_radius : rewire tree 时的半径
RRTStarSearch::RRTStarSearch(float eta, int iter_step, int iter_max, float search_radius) :
eta_(eta),
iter_step_(iter_step),
iter_max_(iter_max),
search_radius_(search_radius)
{
    iter_ = 0;
    map_updated_ = false;
    search_finished_ = false;
    ROS_INFO("\033[1;32m----> this is a [2D] [rrt-star] search method!! .\033[0m");
}

RRTStarSearch::~RRTStarSearch()
{
}

void RRTStarSearch::updateMap(nav_msgs::OccupancyGrid map_data){
    map_ = map_data;
    map_updated_ = true;
}

bool RRTStarSearch::getSearchState(){
    return search_finished_;
}

// 以p_new 为圆心， r 为半径， 在rrt tree中搜索最小代价 父节点
NodeCost RRTStarSearch::NearSearch(geometry_msgs::Point p_new, geometry_msgs::Point p_nearest, 
                                    float r, NodeCostPair tree, nav_msgs::OccupancyGrid map){
    NodeCost p_new_node;
    p_new_node.this_node = p_new;
    float dis_min, dis;
    auto it = tree.find(p_nearest);
    if (it != tree.end())
    {
        dis_min = it->second.cost + Norm(p_new, p_nearest);
        p_new_node.father_node = p_nearest;
        p_new_node.cost = dis_min;
        //std::cout << "p_nearst dis: " << dis_min << std::endl;
    }else
    {
        ROS_ERROR("[NearSearch] point not in rrt_tree");
        return p_new_node;
    }

    int ch;
    for(auto node : tree){
        if (Norm(node.first, p_new) < r)
        {
            ch = CollisionFree(node.first, p_new, map);
            if (ch == 0) // 表示node.first 与p_new之间没有碰撞
            {
                dis = node.second.cost + Norm(p_new, node.first);
                if (dis < dis_min)
                {
                    dis_min = dis;
                    p_new_node.father_node = node.first;
                    p_new_node.cost = dis;
                    //std::cout << "node dis: " << dis_min << std::endl;
                }
            }
            
        }
    }

    return p_new_node;
    
}

//
void RRTStarSearch::RewireTree( NodeCostPair &tree, geometry_msgs::Point p_new, float r, nav_msgs::OccupancyGrid map){

    float dis;
    auto it = tree.find(p_new);
    if (it != tree.end())
    {
        dis = it->second.cost;
    }else
    {
        ROS_INFO("[RewireTree] point not in rrt_tree");
        return;
    }
    
    int ch;
    NodeCost rewire_node;
    for (NodeCostPair::iterator it=tree.begin(); it != tree.end(); it++)
    {
        float dis_node_to_new = Norm(it->first, p_new);
        if (dis_node_to_new < r)
        {
            ch = CollisionFree(p_new, it->first, map);
            if (ch == 0)
            {
                //  p_new的代价 + p_new --> 点p 的代价 <  点p的代价  
                //  p 通过 p_new的代价  <  p点原有的代价 
                if (dis + dis_node_to_new < it->second.cost)
                {
                    rewire_node.this_node = it->first;
                    rewire_node.father_node = p_new;
                    rewire_node.cost = dis + dis_node_to_new;
                    auto ea_it = tree.erase(it);
                    tree.insert(make_pair(rewire_node.this_node, rewire_node));
                }
            }
            
        }
    }
    

}

void RRTStarSearch::search(geometry_msgs::Point p_start, geometry_msgs::Point p_end){
    
    int a, checking;
    p_start_ = p_start;
    p_end_ = p_end;

    NodeCost node_start;
    node_start.cost = 0;
    node_start.this_node = p_start_;
    node_start.father_node = p_start_;
    rrt_tree_set_.insert(make_pair(p_start_, node_start));

    std::queue<geometry_msgs::Point> vis_point_que;
    TicToc search_time;
    while (iter_ < iter_max_ || !search_finished_)
    {
        if (iter_ != 0) // 每次迭代结束处理
        {
            vis_mut_.lock();
            iter_vis_set_.insert(make_pair(iter_, vis_point_que));
            vis_mut_.unlock();
        }

        iter_++;

        while (!vis_point_que.empty())
        {
            vis_point_que.pop();
        }

        TicToc search_time;

        if (!map_updated_)
        {
            ROS_INFO("no map info !! use updateMap function first");
            break;
        }

        if(iter_ % iter_step_ == 0){
            std::cout << "input any to go on!!" << std::endl;
            a = cin.get();
        }


        // step 1: 随机采样一个点
        p_rand_.x = getRandData(map_.info.origin.position.x, map_.info.origin.position.x + map_.info.width*map_.info.resolution);
        p_rand_.y = getRandData(map_.info.origin.position.y, map_.info.origin.position.y + map_.info.height*map_.info.resolution);
        vis_point_que.push(p_rand_); // 储存p_rand用于显示
        ROS_INFO("rand sample a point [ %d ] -- (x %f , y %f)", iter_, p_rand_.x, p_rand_.y);

        // setp 2: 计算rrt_tree上离随机点(p_rand_)最近的点p_nearest_
        p_nearest_ = Near(rrt_tree_set_, p_rand_);
        ROS_INFO("nearest point [ %d ]-- (x %f , y %f)", iter_,  p_nearest_.x, p_nearest_.y);

        // step 3: 沿着p_nearest_ --> p_rand_方向， 以eta为步长，生成新的树节点 p_new_
        p_new_ = Steer(p_nearest_, p_rand_, eta_);
        ROS_INFO("new point [ %d ]-- (x %f , y %f)", iter_,  p_new_.x, p_new_.y);

        // step 4: 碰撞检测 1:占据　0:未占据　-1:未知
        checking = CollisionFree(p_nearest_, p_new_, map_);
        ROS_INFO("the CollisionFree is : %d", checking);

        switch (checking)
        {
            case 1:{
                ROS_INFO("collision");
                break;
            }
            case -1:{
                ROS_INFO("get a unknown area!");
                break;
            }
            case 0:{
                ROS_INFO("get a new point!");

                //将p_new_加入rrt tree
                new_node_ = NearSearch(p_new_, p_nearest_, search_radius_, rrt_tree_set_, map_);
                rrt_tree_set_.insert(make_pair(p_new_, new_node_));

                vis_point_que.push(p_new_);
                vis_point_que.push(new_node_.father_node);

                RewireTree(rrt_tree_set_, p_new_, search_radius_, map_);

                if (nearGoal(p_new_, p_end_))
                {
                    search_finished_ = true;
                    // 将p_end_加入rrt tree
                    NodeCost node_end;
                    node_end.this_node = p_end_;
                    node_end.father_node = p_new_;
                    node_end.cost = new_node_.cost + Norm(p_new_, p_end_);
                    rrt_tree_set_.insert(make_pair(p_end_, node_end));
                    ROS_INFO("get the end points!");
                }
                // break;
                
            }
            
            default:{
                // 回溯一条rrt轨迹
                if (search_finished_)
                {
                    vis_mut_.lock();
                    // 清空p_final_que_
                    while (!p_final_que_.empty())
                    {
                        p_final_que_.pop();
                    }

                    p_final_que_.push(p_end_);

                    while (Norm(p_start_, p_final_que_.back()) > 0.05)
                    {
                        auto find_it = rrt_tree_set_.find(p_final_que_.back());

                        if (find_it != rrt_tree_set_.end())
                        {
                            p_final_que_.push(find_it->second.father_node);
                        }else
                        {
                            ROS_INFO("point not in rrt_tree");
                            break;
                        }
                        std::cout << " dis form start: " << Norm(p_start_, p_final_que_.back()) << std::endl;
                    }
                    if (Norm(p_start_, p_final_que_.back()) < 0.05)
                    {
                        ROS_INFO("back tree sucess!");
                    }

                    vis_mut_.unlock();
                    
                }
            }
        } // switch


    }
    
    ROS_INFO("============= search info ================");
    ROS_INFO("total time is %f ms", search_time.toc());
    ROS_INFO("total sample node nu : %d", iter_);
    ROS_INFO("rrt tree node nu     : %ld", rrt_tree_set_.size());
    ROS_INFO("final node nu        : %ld", p_final_que_.size());
    
}

//****************************************
// 【返回】 std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info
//  iter_vis_info.first 为 算法迭代次数
//  iter_vis_info.second 为 需要显示的点, 存储顺序为 p_rand, p_new, p_nearest, p_new_2, p_nearest_2, ...
//

void RRTStarSearch::getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                                  std::queue<geometry_msgs::Point> &p_final_q,
                                  int &iter){
    
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
            p_final_q.push(p_final_que_.front());
            p_final_que_.pop();
        }
    }
    
    vis_mut_.unlock();
    
    iter = iter_;
}
