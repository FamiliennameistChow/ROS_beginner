/**************************************************************************
 * RRTConnectSearch.hpp
 * 
 * @Author： bornchow
 * @Date: 2022.02.27
 * 
 * @Description:
 *  本程序为rrt系列路径搜索算法中rrt-connect的实现
 * 1. 实现rrt-connect算法
 * 2. 将需要显示的信息反馈
 *  
 *  ****************************************************/
#include "RRTGroupSearch.hpp"
#include "tic_toc.h"
#include <mutex>


class RRTConnectSearch : public RRTGroupSearch
{
private:
    PointPair rrt_tree_set_1_;
    PointPair rrt_tree_set_2_;

    nav_msgs::OccupancyGrid map_;
    bool map_updated_;
    bool search_finished_;
    int iter_;  // 迭代次数

    geometry_msgs::Point p_new_, p_rand_, p_nearest_;
    geometry_msgs::Point p_new_temp_;
    geometry_msgs::Point p_start_, p_end_;

    std::map<int, std::queue<geometry_msgs::Point> > iter_vis_set_;
    std::queue<geometry_msgs::Point> p_final_que_;
    std::mutex vis_mut_;

    // 参数
    float eta_;
    int iter_step_;

private:
    

public:
    RRTConnectSearch(float eta, int iter_step);
    virtual void updateMap(nav_msgs::OccupancyGrid map_data);
    virtual bool getSearchState();
    virtual void search(geometry_msgs::Point p_start, geometry_msgs::Point p_end);
    virtual void getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                                  std::queue<geometry_msgs::Point> &p_final_q,
                                  int &iter);
    ~RRTConnectSearch();
};

//******************************************************
// 【输入】 - eta : 每隔迭代中rrt树扩展的距离 
// 【输入】 - iter_step : 每隔多少步，需要键入以继续
RRTConnectSearch::RRTConnectSearch(float eta, int iter_step) :
eta_(eta),
iter_step_(iter_step)
{
    iter_ = 0;
    map_updated_ = false;
    search_finished_ = false;
    ROS_INFO("\033[1;32m----> this is a [2D] [rrt-connect] search method!! .\033[0m");
}

RRTConnectSearch::~RRTConnectSearch()
{
}

void RRTConnectSearch::updateMap(nav_msgs::OccupancyGrid map_data){
    map_ = map_data;
    map_updated_ = true;
}

bool RRTConnectSearch::getSearchState(){
    return search_finished_;
}

void RRTConnectSearch::search(geometry_msgs::Point p_start, geometry_msgs::Point p_end){
    int a, checking;
    p_start_ = p_start;
    p_end_ = p_end;
    rrt_tree_set_1_.insert(make_pair(p_start_,p_start_));
    rrt_tree_set_2_.insert(make_pair(p_end_, p_end_));

    std::queue<geometry_msgs::Point> vis_point_que;

    while (!search_finished_)
    {
        if (iter_ != 0) // 每次迭代结束处理
        {
            vis_mut_.lock();
            iter_vis_set_.insert(make_pair(iter_, vis_point_que));
            vis_mut_.unlock();

            if (rrt_tree_set_1_.size() > rrt_tree_set_2_.size() ) // 交换V1 与V2, 始终保存rrt_tree_set_1_为较小的树
            {
                rrt_tree_set_1_.swap(rrt_tree_set_2_);
            }
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
            continue;
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
        p_nearest_ = Near(rrt_tree_set_1_, p_rand_);
        ROS_INFO("nearest point [ %d ]-- (x %f , y %f)", iter_,  p_nearest_.x, p_nearest_.y);

        // step 3: 沿着p_nearest_ --> p_rand_方向， 以eta为步长，生成新的树节点 p_new_
        p_new_ = Steer(p_nearest_, p_rand_, eta_);
        ROS_INFO("new point [ %d ]-- (x %f , y %f)", iter_,  p_new_.x, p_new_.y);

        // step 4: 碰撞检测 1:占据　0:未占据　-1:未知
        checking = CollisionFree(p_nearest_, p_new_, map_);
        ROS_INFO("the CollisionFree is : %d", checking);

        if (checking == 1)
        {
            ROS_INFO("collision");
            continue;
        }
        if (checking == -1)
        {
            ROS_INFO("get a unknown area!");
            continue;
        }
        if (checking == 0)
        {
            ROS_INFO("get a new point!");
            //将p_new_加入rrt tree
            rrt_tree_set_1_.insert(make_pair(p_new_, p_nearest_));

            // 将 p_new_， p_nearest_ 储存用于显示
            vis_point_que.push(p_new_);
            vis_point_que.push(p_nearest_);

            p_new_temp_ = p_new_;

            //扩展第二棵树
            // setp 2: 计算rrt_tree2中离 p_new最近的点
            p_nearest_ = Near(rrt_tree_set_2_, p_new_);
            ROS_INFO("nearest point *2* [ %d ]-- (x %f , y %f)", iter_,  p_nearest_.x, p_nearest_.y);

            // step 3: 沿着p_nearest_ --> p_new_方向 以eta为步长，生成新的树节点 p_new_`
            p_new_ = Steer(p_nearest_, p_new_, eta_);
            ROS_INFO("new point *2* [ %d ]-- (x %f , y %f)", iter_,  p_new_.x, p_new_.y);

            // step 4: 碰撞检测 1:占据　0:未占据　-1:未知
            checking = CollisionFree(p_nearest_, p_new_, map_);
            ROS_INFO("the CollisionFree *2* is : %d", checking);

            if (checking == 1)
            {
                ROS_INFO("collision");
                continue;
            }
            if (checking == -1)
            {
                ROS_INFO("get a unknown area!");
                continue;
            }
            if (checking == 0)
            {
                ROS_INFO("get a new point *2*!");
                //将p_new_加入rrt tree
                rrt_tree_set_2_.insert(make_pair(p_new_, p_nearest_));

                // 将 p_new_， p_nearest_ 储存用于显示
                vis_point_que.push(p_new_);
                vis_point_que.push(p_nearest_);

                //贪婪策略
                // step 3: 沿着 p_new_ --> p_new_temp_ 以eta为步长，生成新的树节点 p_new_`
                p_nearest_ = p_new_;
                p_new_ = Steer(p_nearest_, p_new_temp_, eta_);
                ROS_INFO("new point *3* [ %d ]-- (x %f , y %f)", iter_,  p_new_.x, p_new_.y);


                // step 4: 碰撞检测 1:占据　0:未占据　-1:未知
                checking = CollisionFree(p_nearest_, p_new_, map_);
                ROS_INFO("the CollisionFree *3* is : %d", checking);

                if (checking == 1)
                {
                    ROS_INFO("collision");
                    continue;
                }
                if (checking == -1)
                {
                    ROS_INFO("get a unknown area!");
                    continue;
                }
                if (checking == 0)
                {
                    ROS_INFO("get a new point *3*!");
                    //将p_new_加入rrt tree
                    rrt_tree_set_2_.insert(make_pair(p_new_, p_nearest_));

                    // 将 p_new_， p_nearest_ 储存用于显示
                    vis_point_que.push(p_new_);
                    vis_point_que.push(p_nearest_);

                    if (Norm(p_new_, p_new_temp_) < 0.5)
                    {
                        ROS_INFO("Connect !!!");
                        search_finished_ = true;
                        
                        //链接 V1 与 V2
                        rrt_tree_set_1_.insert(make_pair(p_new_, p_new_temp_));

                        //回溯一条rrt轨迹
                        vis_mut_.lock();
                        std::vector<geometry_msgs::Point> p_final_vec;
                        auto find_it = rrt_tree_set_1_.find(p_start_);

                        if (find_it != rrt_tree_set_1_.end()) // p_start 在rrt_tree_set_1_中
                        {
                            p_final_vec.push_back(p_new_); //p_new --> p_start
                            while (Norm(p_start_, p_final_vec.back()) > 0.05)
                            {
                                auto find_it1 = rrt_tree_set_1_.find(p_final_vec.back());
                                if (find_it1 != rrt_tree_set_1_.end())
                                {
                                    p_final_vec.push_back(find_it1->second);
                                }else
                                {
                                    ROS_INFO("point not in rrt_tree");
                                    break;
                                }
                            }

                            reverse(begin(p_final_vec), end(p_final_vec));
                            for(auto vec : p_final_vec){
                                p_final_que_.push(vec);  // p_start --> p_new
                            }

                            while (Norm(p_end_, p_final_que_.back()) > 0.05)
                            {
                                auto find_it2 = rrt_tree_set_2_.find(p_final_que_.back());
                                if (find_it2 != rrt_tree_set_2_.end())
                                {
                                    p_final_que_.push(find_it2->second);
                                }else
                                {
                                    ROS_INFO("point not in rrt_tree");
                                    break;
                                }
                            }
                            
                        }else // p_start在rrt_tree_set_2_中
                        {
                            p_final_vec.push_back(p_new_); //p_new --> p_start
                            while (Norm(p_start_, p_final_vec.back()) > 0.05)
                            {
                                auto find_it1 = rrt_tree_set_2_.find(p_final_vec.back());
                                if (find_it1 != rrt_tree_set_2_.end())
                                {
                                    p_final_vec.push_back(find_it1->second);
                                }else
                                {
                                    ROS_INFO("point not in rrt_tree");
                                    break;
                                }
                            }

                            reverse(begin(p_final_vec), end(p_final_vec));
                            for(auto vec : p_final_vec){
                                p_final_que_.push(vec);  // p_start --> p_new
                            }

                            while (Norm(p_end_, p_final_que_.back()) > 0.05)
                            {
                                auto find_it2 = rrt_tree_set_1_.find(p_final_que_.back());
                                if (find_it2 != rrt_tree_set_1_.end())
                                {
                                    p_final_que_.push(find_it2->second);
                                }else
                                {
                                    ROS_INFO("point not in rrt_tree");
                                    break;
                                }
                            }
                        }

                        vis_mut_.unlock();
                    }
                    
                }
            }
        } // 第一棵树扩展的碰撞检测


    }
    

}

//****************************************
// 【返回】 std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info
//  iter_vis_info.first 为 算法迭代次数
//  iter_vis_info.second 为 需要显示的点, 存储顺序为 p_rand, p_new, p_nearest, p_new_2, p_nearest_2, ...
//  
void RRTConnectSearch::getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
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


