 /********************************************************
 * trajectory_server.cpp
 * 
 *
 * Author： Born Chow
 * Date: 2021.05.07
 *
 * 说明：scout 小车轨迹服务器
 * 
 * 接收轨迹规划的结果，发布控制指令
 *
 * 【订阅】
 *
 * 【发布】
 *
 ******************************************************/

#include "ros/ros.h"
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h> //转换函数头文件
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/GetModelState.h>

#include <thread>

// #define SHOW_GT_TRAJ
#define PI 3.1415926

using namespace std;

class TrajectoryServer
{
private:
    ros::NodeHandle n_;

    ros::Subscriber odom_sub_;
    ros::Subscriber traj_sub_;

    ros::Publisher cmd_pub_;
    ros::Publisher track_state_pub_;
    ros::Publisher vis_exe_traj_pub_;


    // ros 参数
    std::string odom_topic_;
    double vehicle_vel_;
    double vehicle_length_;
    double ld_k_;
    double ld_c_;
    double escape_v_;


    geometry_msgs::Point vehicle_pos_;
    double vehicle_roll_, vehicle_pitch_, vehicle_yaw_;
    vector<geometry_msgs::Transform> traj_trans_list_;
    bool wapoint_change_;
    std_msgs::Float32 traj_track_state_; //轨迹跟踪状态, 代表剩余轨迹比例 --> 0.0 表示追踪结束 1 表示轨迹跟踪开始，等待跟踪开始
    geometry_msgs::Point pt_last;

    visualization_msgs::Marker traj_vis_; //用于显示真实小车的轨迹

    #ifdef SHOW_GT_TRAJ
    ros::Publisher vis_gt_traj_pub_;
    ros::ServiceClient vehicle_state_client_;
    //　gazbeo真实轨迹显示  由于slam有误差弃用
    gazebo_msgs::GetModelState gazebo_model_state_;
    #endif


private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void trajCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg);

    int nearPoint(geometry_msgs::Point vehicle_pos);
    float call_dis(geometry_msgs::Point p1, geometry_msgs::Point p2);

public:
    TrajectoryServer(ros::NodeHandle& nh, ros::NodeHandle &private_nh);
    void TrajFollow();

    void pubExecuteTraj(geometry_msgs::Point pt); //发布小车真实走过的轨迹

    #ifdef SHOW_GT_TRAJ
    void visualizeGTtrajThread();
    #endif
    ~TrajectoryServer();
};

TrajectoryServer::TrajectoryServer(ros::NodeHandle& nh, ros::NodeHandle &private_nh):
n_(nh)
{
    // 读取参数服务器
    private_nh.param<string>("odom_topic", odom_topic_, "/integrated_to_init");
    private_nh.param<double>("Vel", vehicle_vel_, 1.0);
    private_nh.param<double>("Length", vehicle_length_, 1.0);
    private_nh.param<double>("ld_k", ld_k_, 1.0);  // 　前视距离　ld = ld_k_ * vehicle_vel_ + ld_c_
    private_nh.param<double>("ld_c", ld_c_, 0.5);  // 　前视距离　越大，跟踪越平滑, 但跟踪精度下降; 越小 跟踪精度越高,但越震荡
    private_nh.param<double>("escape_v", escape_v_, 0.3); //　逃逸速度


    odom_sub_ = n_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &TrajectoryServer::odomCallback, this);
    traj_sub_ =n_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("waypoints", 1, &TrajectoryServer::trajCallback, this);

    cmd_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    track_state_pub_ = n_.advertise<std_msgs::Float32>("/traj_track_state", 1000);  // 发布轨迹跟踪状态

    vis_exe_traj_pub_ = n_.advertise<visualization_msgs::Marker>("/exe_trajectory_vis", 100);  // 发布gazebo真实轨迹

    #ifdef SHOW_GT_TRAJ
    vis_gt_traj_pub_ = n_.advertise<visualization_msgs::Marker>("/gt_trajectory_vis", 100);  // 发布gazebo真实轨迹
    vehicle_state_client_ = n_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); //获取gazebo中机器人的状态
    gazebo_model_state_.request.model_name = "scout/"; 
    gazebo_model_state_.request.relative_entity_name = "world";
    #endif

    pt_last.x = 9999.0;
    pt_last.y = 9999.0;
    pt_last.z = 9999.0;

    traj_vis_.header.stamp       = ros::Time::now();
    traj_vis_.header.frame_id    = "map"; 

    traj_vis_.ns = "exe_trajectory";
    traj_vis_.id = 1;
    traj_vis_.type = visualization_msgs::Marker::SPHERE_LIST;

	traj_vis_.action = visualization_msgs::Marker::ADD;
    traj_vis_.scale.x = 0.25;
    traj_vis_.scale.y = 0.25;
    traj_vis_.scale.z = 0.25;
    traj_vis_.pose.orientation.x = 0.0;
    traj_vis_.pose.orientation.y = 0.0;
    traj_vis_.pose.orientation.z = 0.0;
    traj_vis_.pose.orientation.w = 1.0;
    traj_vis_.color.r = 1.0;
    traj_vis_.color.g = 1.0;
    traj_vis_.color.b = 0.0;
    traj_vis_.color.a = 1.0;

}

TrajectoryServer::~TrajectoryServer()
{

}


void TrajectoryServer::odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    vehicle_pos_.x = msg->pose.pose.position.x;
	vehicle_pos_.y = msg->pose.pose.position.y;
	vehicle_pos_.z = msg->pose.pose.position.z;

    // 欧拉角转四元数 https://blog.csdn.net/qq_23670601/article/details/87968936
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);

    tf::Matrix3x3(quat).getRPY(vehicle_roll_, vehicle_pitch_, vehicle_yaw_);//进行转换

    // pubExecuteTraj(vehicle_pos_);

}

void TrajectoryServer::trajCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg){
    // 获取轨迹中的位姿信息
    traj_trans_list_.clear();
    for (int i = 0; i < msg->points.size(); i++)
    {
        traj_trans_list_.push_back(msg->points[i].transforms[0]);
    }

    wapoint_change_ = true;

}


// 找到轨迹上离小车最近的点 暂时未用
int TrajectoryServer::nearPoint(geometry_msgs::Point vehicle_pos){
    double dis_min = 999999.0;
    double dis;
    int index;
    for (int i = 0; i <traj_trans_list_.size(); i++)
    {
        dis = pow(traj_trans_list_[i].translation.x - vehicle_pos.x, 2) + pow(traj_trans_list_[i].translation.y - vehicle_pos.y, 2);
        if (dis < dis_min)
        {
            dis_min = dis;
            index = i;
        } 
    }
    return index;
}


//计算两点间的距离
float TrajectoryServer::call_dis(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return pow( pow(p1.x - p2.x, 2) + pow(p1.y - p2.y , 2) + pow(p1.z - p2.z, 2), 0.5);
}

void TrajectoryServer::pubExecuteTraj(geometry_msgs::Point pt){

    if (call_dis(pt, pt_last) > 0.05)
    {
        traj_vis_.points.push_back(pt);
        pt_last = pt;
        vis_exe_traj_pub_.publish(traj_vis_);
        // cout << "-------" << endl;
    }
}

# ifdef SHOW_GT_TRAJ
void TrajectoryServer::visualizeGTtrajThread(){
	ros::Rate rate(5);
    geometry_msgs::Point pt;

    traj_vis_.header.stamp       = ros::Time::now();
    traj_vis_.header.frame_id    = "world"; //gazebo输出的坐标是全局world坐标系上的

    traj_vis_.ns = "gt_trajectory";
    traj_vis_.id = 1;
    traj_vis_.type = visualization_msgs::Marker::SPHERE_LIST;

	traj_vis_.action = visualization_msgs::Marker::ADD;
    traj_vis_.scale.x = 0.15;
    traj_vis_.scale.y = 0.15;
    traj_vis_.scale.z = 0.15;
    traj_vis_.pose.orientation.x = 0.0;
    traj_vis_.pose.orientation.y = 0.0;
    traj_vis_.pose.orientation.z = 0.0;
    traj_vis_.pose.orientation.w = 1.0;
    traj_vis_.color.r = 1.0;
    traj_vis_.color.g = 1.0;
    traj_vis_.color.b = 0.0;
    traj_vis_.color.a = 0.6;

	while (ros::ok())
	{
        vehicle_state_client_.call(gazebo_model_state_);
        bool su = gazebo_model_state_.response.success;
        
        if (su)
        {
            pt = gazebo_model_state_.response.pose.position;

            if (call_dis(pt, pt_last) > 0.1)
            {
                traj_vis_.points.push_back(pt);
                pt_last = pt;
                vis_gt_traj_pub_.publish(traj_vis_);
                cout << "-------" << su << endl;
            }
        }

		rate.sleep();

	}
	
}
#endif

void TrajectoryServer::TrajFollow(){
    //pure pursuit controller
    int index  = 0;
    double dis, dis_end_to_me, dis_start_to_end;
    int index_last = 1;

    // ld 前视距离　越大，跟踪越平滑, 但跟踪精度下降; 越小 跟踪精度越高,但越震荡
    double ld = ld_k_ * vehicle_vel_ + ld_c_;
    int count = 0;
    wapoint_change_ = false;
    ros::Rate rate(50.0);

    // cout << "traj size: " <<  traj_trans_list_.size() << endl;
    if (traj_trans_list_.empty())
    {
        return;
    }

    // cout << "2222" << endl;
    // cout << "vehicle pos: " << vehicle_pos_.x << " " << vehicle_pos_.y << endl;
    

    while (ros::ok())
    {
        // cout << " change : " << wapoint_change_ << endl;

        if (wapoint_change_){
    
            break;

        }
        
        count++;

        dis_end_to_me = pow(pow(traj_trans_list_[traj_trans_list_.size()-1].translation.x - vehicle_pos_.x, 2) + pow(traj_trans_list_[traj_trans_list_.size()-1].translation.y - vehicle_pos_.y, 2), 0.5);
        // cout << "dis_end_to_me: " << dis_end_to_me << endl;
        dis_start_to_end = pow(pow(traj_trans_list_[traj_trans_list_.size()-1].translation.x - traj_trans_list_[1].translation.x, 2) + 
                                pow(traj_trans_list_[traj_trans_list_.size()-1].translation.y - traj_trans_list_[1].translation.y, 2), 0.5);
        
        // cout << "dis_start_to_end:  " << dis_start_to_end << endl;
        if (dis_start_to_end < 0.1) // 无人车卡住， 逃逸策略
        {
            geometry_msgs::Twist cmd;
            cmd.linear.x = escape_v_;
            cmd.angular.z = 0.0;
            cmd_pub_.publish(cmd);

            traj_track_state_.data = 0.0;
            track_state_pub_.publish(traj_track_state_);
            cout << "[traj server]: stack" << endl;
            break;
        }
        

        if (dis_end_to_me < 0.5) // 追踪结束
        {
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_.publish(cmd);

            traj_track_state_.data = 0.0;
            track_state_pub_.publish(traj_track_state_);
            cout << "[traj server]: traj follow end" << endl;
            break;
        }


        if (count == 1)
        {
            index = nearPoint(vehicle_pos_);
        }
        
        
        if (dis_end_to_me < ld) //追踪终止点
        {
            index = traj_trans_list_.size() -1;
        }else // 否则寻找下一个追踪点
        {
            for(int i=index; i <traj_trans_list_.size(); i++){
                dis = pow(pow(traj_trans_list_[i].translation.x - vehicle_pos_.x, 2) + pow(traj_trans_list_[i].translation.y - vehicle_pos_.y, 2), 0.5);
                
                if (dis > ld)
                {
                    index = i;
                    break;
                }
            }
        }
        
        

        //  参考点与车质点方向 与 车航向角之前的差; 
        //  如果参考点在vehicle左边 theta [0, pi] , 反之则为[0, -pi]
        double theta = atan2(traj_trans_list_[index].translation.y-vehicle_pos_.y, traj_trans_list_[index].translation.x-vehicle_pos_.x) - vehicle_yaw_;

        // double delta = atan2(2.0 * vehicle_length_ * sin(theta) / ld, 1.0);

        // tan(delta) = Vy / V --> Vy = tan(theta)*V
        // Vy = omega * vehicle_length_ --> omega(角速度) = tan(theta)*V / vehicle_length_
        // double omega = tan(delta) * vehicle_vel_ / vehicle_length_;

        double omega = 2.0 * vehicle_length_ * sin(theta) * vehicle_vel_ / (vehicle_length_ * ld);

        // 发布控制命令
        geometry_msgs::Twist cmd;
        if (abs(theta) > PI / 8)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = omega;
        }else
        {
            cmd.linear.x = vehicle_vel_;
            cmd.angular.z = omega;
        }

        cmd_pub_.publish(cmd);

        traj_track_state_.data = dis_end_to_me / dis_start_to_end;
        track_state_pub_.publish(traj_track_state_);

        if (index != index_last)
        {
            cout << "vehicle pos: " << vehicle_pos_.x << " " << vehicle_pos_.y << " " << vehicle_yaw_ << endl;
            cout << "traj_trans_list_[index]: " << index << " of " << traj_trans_list_.size() << " "<< traj_trans_list_[index].translation.x << " " << traj_trans_list_[index].translation.y << endl;
            cout << "traj_track_state: " << traj_track_state_ << endl; 
            cout << " left -> [0, pi] or  [0, -pi] " << endl;
            cout << " theta " << theta << endl;
            // cout << " delta " << delta << endl;
            cout << " omega " << omega << endl;
        }

        index_last = index;
    
        ros::spinOnce();
        rate.sleep();
    }

}



// ---------------------------------------------------------------------------------
/// class end

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajServer");
    ROS_INFO("\\033[1;32m---->\\033[0m trajServer Node Started.");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate rate(30.0);

    TrajectoryServer trajServer(nh, nh_);

    #ifdef SHOW_GT_TRAJ
    std::thread gt_visthread(&TrajectoryServer::visualizeGTtrajThread, &trajServer);
    #endif

    while (ros::ok())
    {
        ros::spinOnce();

        trajServer.TrajFollow();
        
        rate.sleep();
    }
    

    ros::spin();


    return 0;

}