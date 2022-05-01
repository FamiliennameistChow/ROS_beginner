 /********************************************************
 *  接收轨迹规划的结果，发布路径跟踪的控制指令
 *
 *  订阅 类型：nav_msgs::Odometry  
 *      话题：gazebo仿真真值： /ground_truth/odom  
 *           SLAM包的估计值： /odometry/imu
 *      类型：nav_msgs::Path 
 *      话题：/path_planned
 *
 *  发布 类型：ackermann_msgs::AckermannDrive
 *      话题：/cmd_vel
 *
 ******************************************************/

#include <cmath>
#include <thread>
#include <iomanip>
#include <iostream>
#include <string.h>
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>          //转换函数头文件
#include <ackermann_msgs/AckermannDrive.h>

using namespace std;

class TrajectoryServer
{
private:
    ros::NodeHandle n_;
    ros::Subscriber odom_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher  cmd_pub_;
    ros::Publisher  track_state_pub_;

    // ros 参数
    std::string odom_topic_;
    double vehicle_vel_;
    double vehicle_length_;
    double ld_k_;
    double ld_c_;
    double escape_v_;

    ackermann_msgs::AckermannDrive cmd_;
    geometry_msgs::Point vehicle_position_;
    double vehicle_roll_, vehicle_pitch_, vehicle_yaw_;
    std::vector<geometry_msgs::Pose> path_;
    bool wapoint_change_;
    std_msgs::Float32 traj_track_state_;   //轨迹跟踪状态, 代表剩余轨迹比例 --> 0.0 表示追踪结束 1 表示轨迹跟踪开始，等待跟踪开始

private:
    void  odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void  trajCallback(const nav_msgs::Path::ConstPtr &msg);
    int   nearPoint(geometry_msgs::Point position);
    float call_dis(geometry_msgs::Point p1, geometry_msgs::Point p2);
    void  show_msg(string s);
    void  show_state(int index, bool isErase);

public:
    TrajectoryServer(ros::NodeHandle& nh, ros::NodeHandle &private_nh);
    ~TrajectoryServer(){}; 

    void PurePursuit();   
    void MPC_Runnable(); 
};

TrajectoryServer::TrajectoryServer(ros::NodeHandle& nh, ros::NodeHandle &private_nh):
n_(nh)
{
    // 读取参数服务器
    // n_.param<string>("/laser_odom_potic", odom_topic_, "/odometry/imu");   //  使用 SLAM 包估计时使用 /odometry/imu 里程计
    n_.param<string>("/laser_odom_potic", odom_topic_, "/ground_truth/odom"); //  gazebo 的场景真值，验证控制算法
    private_nh.param<double>("Vel", vehicle_vel_, 1.0);
    private_nh.param<double>("Length", vehicle_length_, 2.054);
    private_nh.param<double>("ld_k", ld_k_, 1.0);  // 　前视距离　ld = ld_k_ * vehicle_vel_ + ld_c_
    private_nh.param<double>("ld_c", ld_c_, 0.5);  // 　前视距离　越大，跟踪越平滑, 但跟踪精度下降; 越小 跟踪精度越高,但越震荡
    private_nh.param<double>("escape_v", escape_v_, 0.3); //　逃逸速度

    odom_sub_ = n_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &TrajectoryServer::odomCallback, this);
    // path_sub_ = n_.subscribe<nav_msgs::Path>("/move_base_node/TrajectoryPlannerROS/local_plan", 1, &TrajectoryServer::trajCallback, this);
    path_sub_ = n_.subscribe<nav_msgs::Path>("/path_planned", 1, &TrajectoryServer::trajCallback, this);

    cmd_pub_ = n_.advertise<ackermann_msgs::AckermannDrive>("/cmd_vel", 30);
    track_state_pub_ = n_.advertise<std_msgs::Float32>("/traj_track_state", 1000);              // 发布轨迹跟踪状态

    cmd_.steering_angle_velocity = 2.0;
    cmd_.steering_angle          = 0.0;
    cmd_.acceleration            = 2.0;
    cmd_.jerk                    = 0.0;
    cmd_.speed                   = 0.0;

    show_msg(("odom: " + odom_topic_).c_str());
}

void TrajectoryServer::show_msg(string s){
    ROS_INFO(("[TrajectoryServer]: " + s).c_str());
}

void TrajectoryServer::show_state(int index, bool isErase){
    if(isErase){
        std::cout<<" "<<std::endl;
        std::cout << "[Running] **********************************************************************\n"
        << "[TrajectoryServer]: " << index << " of " << path_.size() << " "
        << "   vehicle pos: " <<setiosflags(ios::fixed)<<setprecision(2)<< vehicle_position_.x << " " 
        <<setiosflags(ios::fixed)<<setprecision(2)<< vehicle_position_.y 
        <<setiosflags(ios::fixed)<<setprecision(2)<< " " << vehicle_yaw_ <<" \n"
        << "[TrajectoryServer]: track_state: "<<setiosflags(ios::fixed)<<setprecision(2)
        << "["<<string((40*(1.0-traj_track_state_.data)), '>') << string((40*(traj_track_state_.data)), ' ')
        <<" ] "<< 100-traj_track_state_.data *100.0 <<" %             "<<"\033[3A";
    }else{
        std::cout<<" "<<std::endl;
        std::cout << "[Running] **********************************************************************\n"
        << "[TrajectoryServer]: " << index << " of " << path_.size() << " "
        << "   vehicle pos: " <<setiosflags(ios::fixed)<<setprecision(2)<< vehicle_position_.x << " " 
        <<setiosflags(ios::fixed)<<setprecision(2)<< vehicle_position_.y << " " 
        <<setiosflags(ios::fixed)<<setprecision(2)<< vehicle_yaw_ <<" \n"
        << "[TrajectoryServer]: track_state: "<<setiosflags(ios::fixed)<<setprecision(2)
        << "["<<string(40, '>') <<" ] "<< 100.0 <<" %             "<<std::endl<<std::endl;
    }
}

void TrajectoryServer::odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    vehicle_position_.x = msg->pose.pose.position.x;
	vehicle_position_.y = msg->pose.pose.position.y;
	vehicle_position_.z = msg->pose.pose.position.z;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(vehicle_roll_, vehicle_pitch_, vehicle_yaw_);
}

void TrajectoryServer::trajCallback(const nav_msgs::Path::ConstPtr &msg){
    path_.clear();
    for(geometry_msgs::PoseStamped poseStamped : msg->poses){
        path_.push_back(poseStamped.pose);
    }
    wapoint_change_ = true;
}

//计算两点间的距离
float TrajectoryServer::call_dis(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return pow( pow(p1.x - p2.x, 2) + pow(p1.y - p2.y , 2) + pow(p1.z - p2.z, 2), 0.5);
}

float _call_dis(geometry_msgs::Point p1, geometry_msgs::Pose p2){
    return pow( pow(p1.x - p2.position.x, 2) + pow(p1.y - p2.position.y , 2) + pow(p1.z - p2.position.z, 2), 0.5);
}
int TrajectoryServer::nearPoint(geometry_msgs::Point position){
    float dis = _call_dis(position, path_[0]);
    int i = 0;
    int index = 0;
    for(auto pose : path_){
        float _dis = _call_dis(position, pose);
        if(_dis < dis){
            dis = _dis;
            index = i;
        }
        i++;
    }
    return index;
}

void TrajectoryServer::PurePursuit(){
    ros::spinOnce();
    int index  = 0;
    double dis, dis_end_to_me, dis_start_to_end;
    int index_last = 1;

    // ld 前视距离　越大，跟踪越平滑, 但跟踪精度下降; 越小 跟踪精度越高,但越震荡
    double ld = ld_k_ * vehicle_vel_ + ld_c_;
    int count = 0;
    wapoint_change_ = false;
    ros::Rate rate(50.0);

    if (path_.empty())
    {
        return;
    }
    while (ros::ok())
    {
        if (wapoint_change_){
            break;
        }

        count++;

        dis_end_to_me    = pow(pow(path_[path_.size()-1].position.x - vehicle_position_.x, 2) + 
                               pow(path_[path_.size()-1].position.y - vehicle_position_.y, 2), 0.5);
        dis_start_to_end = pow(pow(path_[path_.size()-1].position.x - path_[0].position.x, 2) + 
                               pow(path_[path_.size()-1].position.y - path_[0].position.y, 2), 0.5);

        if (dis_start_to_end < 0.1) // 无人车卡住， 逃逸策略
        {
            cmd_.speed = escape_v_;
            cmd_.steering_angle = 0.0;
            cmd_pub_.publish(cmd_);
            traj_track_state_.data = 0.0;
            track_state_pub_.publish(traj_track_state_);
            ROS_INFO_STREAM_THROTTLE(1,"[TrajectoryServer]: stack");
            break;
        }

        if (dis_end_to_me < 0.2) // 追踪结束
        {
            cmd_.speed = 0.0;
            cmd_.steering_angle = 0.0;
            cmd_pub_.publish(cmd_);

            traj_track_state_.data = 0.0;
            track_state_pub_.publish(traj_track_state_);
            show_state(path_.size(), false);
            ROS_INFO_STREAM("[TrajectoryServer]: traj follow end");
            path_.clear();
            break;
        }

        if (count == 1)
        {
            index = nearPoint(vehicle_position_);
        }

        if (dis_end_to_me < ld) //追踪终止点
        {
            index = path_.size() -1;
        }else // 否则寻找下一个追踪点
        {
            for(int i=index; i <path_.size(); i++){
                dis = pow(pow(path_[i].position.x - vehicle_position_.x, 2) + pow(path_[i].position.y - vehicle_position_.y, 2), 0.5);
                
                if (dis > ld)
                {
                    index = i;
                    break;
                }
            }
        }
        
        //  参考点与车质点方向 与 车航向角之前的差; 
        //  如果参考点在vehicle左边 theta [0, pi] , 反之则为[0, -pi]
        double theta = atan2(path_[index].position.y-vehicle_position_.y, path_[index].position.x-vehicle_position_.x) - vehicle_yaw_;

        double delta = atan2(2.0 * vehicle_length_ * sin(theta) / ld, 1.0);

        if (abs(theta) > M_PI/2)
        {
            cmd_.speed = -vehicle_vel_;
            cmd_.steering_angle = delta * 180 * M_1_PI;
        }else
        {
            cmd_.speed = vehicle_vel_;
            cmd_.steering_angle = delta * 180 * M_1_PI;
        }

        // cmd_.speed = vehicle_vel_;
        // cmd_.steering_angle = delta * 180 * M_1_PI;

        cmd_pub_.publish(cmd_);

        traj_track_state_.data = dis_end_to_me / dis_start_to_end;
        track_state_pub_.publish(traj_track_state_);

        if (index != index_last)
        {
            show_state(index, true);
        }

        index_last = index;
        ros::spinOnce();
        rate.sleep();
    }
}

// TODO: 最好是把这个控制器的方法写在外面, 要不然这个类里面矩阵太多了。
void TrajectoryServer::MPC_Runnable(){
    

}

// ---------------------------------------------------------------------------------
/// class end

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajServer");
    ROS_INFO("[TrajectoryServer]: Node Started.");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate loop_rate(30);
    TrajectoryServer trajServer(nh, nh_);
    while(ros::ok()){
        ros::spinOnce();
        trajServer.PurePursuit();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}