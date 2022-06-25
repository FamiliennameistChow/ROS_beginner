/**************************************************************************
 * RRTSearch.hpp
 * 
 * @Author： bornchow
 * @Date: 2022.02.25
 * 
 * @Description:
 *  本程序为rrt系列路径搜索算法中原始rrt的实现
 * 1. 实现原始选取rrt算法
 * 2. 将需要显示的信息反馈
 *  
 *  ****************************************************/
#include "RRTGroupSearch.hpp"
#include "tic_toc.h"
#include <mutex>

// class RRTSearch

class RRTSearch : public RRTGroupSearch
{
private:
    PointPair rrt_tree_set_; 
    nav_msgs::OccupancyGrid map_;
    bool map_updated_;
    bool search_finished_;
    int iter_; // 计数迭代次数
    geometry_msgs::Point p_new_, p_rand_, p_nearest_;
    geometry_msgs::Point p_start_, p_end_;

    std::map<int, std::queue<geometry_msgs::Point> > iter_vis_set_;
    std::queue<geometry_msgs::Point> p_final_que_;
    std::mutex vis_mut_;

    // 参数
    float eta_;
    int iter_step_;

public:
    RRTSearch(float eta, int iter_step); 
    virtual void updateMap(nav_msgs::OccupancyGrid map_data);
    virtual void search(geometry_msgs::Point p_start, geometry_msgs::Point p_end);
    virtual bool getSearchState();
    virtual void getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                                  std::queue<geometry_msgs::Point> &p_final_q,
                                  int &iter);
    ~RRTSearch();
};

//******************************************************
// 【输入】 - eta : 每隔迭代中rrt树扩展的距离 
// 【输入】 - iter_step : 每隔多少步，需要键入以继续
RRTSearch::RRTSearch(float eta, int iter_step) :
eta_(eta),
iter_step_(iter_step)
{
    iter_ = 0;
    map_updated_ = false;
    search_finished_ = false;
    ROS_INFO("\033[1;32m----> this is a [2D] [rrt] search method!! .\033[0m");
}


RRTSearch::~RRTSearch()
{
}

void RRTSearch::updateMap(nav_msgs::OccupancyGrid map_data){
    map_ = map_data;
    map_updated_ = true;
}


void RRTSearch::search(geometry_msgs::Point p_start, geometry_msgs::Point p_end){

    int a, checking;
    p_start_ = p_start;
    p_end_ = p_end;
    rrt_tree_set_.insert(make_pair(p_start_,p_start_));

    std::queue<geometry_msgs::Point> vis_point_que;
    // std::cout << " ------ " << std::endl;
    // for(auto &p : rrt_tree_set_){
    //     std::cout << "{--- " << p.first << " : " << p.second << "---} " << std::endl;
    // }
    // std::cout << " ------ " << std::endl;

    while (!search_finished_)
    {
        if (iter_ != 0) // 每次迭代结束处理
        {
            std::lock_guard<std::mutex> mut_locker(vis_mut_);
            iter_vis_set_.insert(make_pair(iter_, vis_point_que));
        
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
            rrt_tree_set_.insert(make_pair(p_new_, p_nearest_));

            // 将 p_new_， p_nearest_ 储存用于显示
            vis_point_que.push(p_new_);
            vis_point_que.push(p_nearest_);

            //判断p_new_是否与终点接近
            if (nearGoal(p_new_, p_end_))
            {
                //将p_end_加入rrt tree
                rrt_tree_set_.insert(make_pair(p_end_, p_new_));
                ROS_INFO("you have got the goal!----");

                // 回溯一条rrt路径
                std::lock_guard<std::mutex> mut_locker(vis_mut_);
                p_final_que_.push(p_end_);
                while (Norm(p_start_, p_final_que_.back()) > 0.05)
                {
                    auto find_it = rrt_tree_set_.find(p_final_que_.back());
                    if ( find_it != rrt_tree_set_.end())
                    {
                        p_final_que_.push(find_it->second);
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
    
                search_finished_ = true;

                ROS_INFO("============= search info ================");
                ROS_INFO("total time is %f ms", search_time.toc());
                ROS_INFO("total sample node nu : %d", iter_);
                ROS_INFO("rrt tree node nu     : %ld", rrt_tree_set_.size());
                ROS_INFO("final node nu        : %ld", p_final_que_.size());
            }
            
        }

    }
    
}

//****************************************
// 【返回】 std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info
//  iter_vis_info.first 为 算法迭代次数
//  iter_vis_info.second 为 需要显示的点, 存储顺序为 p_rand, p_new, p_nearest, p_new_2, p_nearest_2, ...
//
void RRTSearch::getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                                  std::queue<geometry_msgs::Point> &p_final_q,
                                  int &iter){
    
    std::lock_guard<std::mutex> mut_locker(vis_mut_);
    
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
    
    
    
    iter = iter_;
}

bool RRTSearch::getSearchState(){
    return search_finished_;
}
