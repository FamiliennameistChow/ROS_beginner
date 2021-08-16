/*
 * @Author: Haojie X
 * @Date: 2021-03-31 08:52:42
 * @LastEditTime: 2021-04-14 19:48:38
 * @LastEditors: Please set LastEditors
 * @Description: 侦察机代码
 * @FilePath: /target_landing/src/scout_plane1.cpp
 */
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <algorithm>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <ros/console.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <dark/Poerro.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/ObjectCount.h"
#include "drone_flight_modes.hpp"

#include <target_landing/scout_plan_msg.h>
#include <target_landing/AttackerMsg.h>

// get status of attackers

namespace Status_n {
enum Status_t {
	INIT_MISSION, //任务初始化
	BIDDING, // 投标
	CONFIRM, //确认
	WINBID, //执行,中标
	ENDING //结束
};
}

namespace my_Status_n {
enum my_Status_t {
	SEARCH, //寻找目标  
	TOCENTER, // 对中,不停发布对中速度
	ENDING //结束
};
}

typedef Status_n::Status_t Status;
typedef my_Status_n::my_Status_t my_Status;

typedef struct mission_type
{
    int nav_num;
    sensor_msgs::NavSatFix GPSinfo;
}mission_type;

bool cmp(const pair<int, float>& a, const pair<int, float>& b) {

    return a.second <= b.second;
    
}

class scout_plane
{
private:
    
    //------------------------------------>>>>>>>>>>>>parameter<<<<<<<<<<<<<<<----------------------------
    cv::Mat image_,image_display_;//图像变量

    int target_val_; //检测出单个目标的数字大小
    int target_count_; //检测出目标的个数

    target_landing::scout_plan_msg scout_message_;//涉及到线程变量修改,其中有一个成员在主线程里修改了,其余的在新建线程

    vector<double> dis_tocenter_;//存每一个目标对应的Y值

    //flag
    bool reach_flag_ = false; //判断侦查无人机是否已经到达了中心点,如果到达,GPS赋值并发送
    double pubGPStime_;// 发布GPS信息时的时间戳
    double current_time_;// 现在时间的时间戳,用来计算等待cost返回来的间隔
    double dt_;// 时间间隔
    int see_count_;


    int the_value_of_the_num_;    // 检测到的数字大小

    bool see_you;    //是否有目标出现
    
    
    map<int, float> cost_result_;    //比较cost结果的map
    map<int, float>::iterator iter_;
    
    //无人机速度
    geometry_msgs::Twist cmd_vel_; //速度变量
    double tocenter_vx_ = 0;//对中的无人机X速度
    double tocenter_vy_ = 0;//对中的无人机Y速度
    double last_vx_;//无人机从search切换到center前的速度值
    double vx_ = 0;
    double start_vx_= 0;
    double Kp_, Ki_; //PID参数
    double last_err_x_ = 10;//无人机上一循环飞行的误差,考虑到误差突变对速度的影响
    float err_x_ = 10; // 无人机的前进方向误差值
    float err_y_; // 无人机的误差值y
    double single_distocen_; //逼近目标和中心点的差值
    int uav_num; // 无人机启动编号0-1
    double error_sum_x_; //ki_的误差累计值x----------------------------------还没用到
    double error_sum_y_; //ki_的误差累计值y----------------------------------还没用到
    double uav_crr_height_; //无人机当前的高度值
    float Thres_distance_land; // 判定到达中心的阈值

    float fx,fy,cx,cy; // 相机参数

    std::mutex status_Buf_;//线程锁-------------------------------------------还没用到

    //Drone
    AeroDrone* detectDrone_; //无人机
    my_Status current_status_; //目前的状态
    
    // Topics

    std::string target_GPS_topic_;

    std::string target_Dronenum_topic_;

    std::string attacker_cost_status_dronenum_topic_;

    std::string scout_plane_GPSpos_topic_;

    std::string target_postion_from_image_topic_;

    std::string target_count_from_image_topic_;
    
    //Data
    sensor_msgs::NavSatFix target_GPS;
    sensor_msgs::NavSatFix scout_plane_GPS;

    // Image Subscriber and Publisher
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    //Ros SUBSCRIBERS
    //攻击机返回,cost\攻击机无人机编号\状态
    ros::Subscriber attacker_exec_result_sub_;
    //mavros,订阅侦查无人机目前的GPS信息
    ros::Subscriber scout_curr_GPSPOS_sub_;
    //目标的信息
    ros::Subscriber target_pos_info_sub_;
    ros::Subscriber target_count_info_sub_;

    //Ros PUBLISHERS,目标的GPS信息,发给攻击机和编号,并计算cost值最小的无人机
    ros::Publisher target_GPS_pos_pub_;

    ros::Publisher target_dronenum_pub_;

    void setsearchConfig();
    
    void setTOCENTERconfig();

    void setPUBGPSconfig();

    void setPUBGNUMconfig();     

public:
    scout_plane(ros::NodeHandle& nh, ros::NodeHandle &nh_);

    void GPSgetCallback(const sensor_msgs::NavSatFixConstPtr& GPSpos_msg);//获取GPS位置

    void target_pos_infocallback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);//获取检测目标的位置信息

    void target_count_infocallback(const darknet_ros_msgs::ObjectCountConstPtr &msg);

    void attackerresultinfocallback(const target_landing::AttackerMsgConstPtr& msg);//订阅无人机发布的cost

    //计算到图像中心的距离,这里只有Y
    double cal_disto_imgcenter(const double & xmax,const double & xmin,const double & ymax,const double & xymin);

    cv::Point2d fromcam_to_uav(const darknet_ros_msgs::BoundingBox &msg, double uav_crr_height_);
    
    void run();

    void cost_sort();

    void mission_Thread();//新建线程入口
    
    void mission_make();//线程函数入口

    ~scout_plane();//析构函数
};

scout_plane::scout_plane(ros::NodeHandle& nh, ros::NodeHandle &nh_)
{

    pubGPStime_ = std::numeric_limits<double>::quiet_NaN();
    current_time_ = std::numeric_limits<double>::quiet_NaN();
    see_count_ = 0;
    single_distocen_ = 999;
    see_you = false;

    //话题参数读取
    nh_.getParam("target_GPS_topic_",target_GPS_topic_);

    nh_.getParam("target_Dronenum_topic_",target_Dronenum_topic_);
 
    nh_.getParam("attacker_cost_status_dronenum_topic_",attacker_cost_status_dronenum_topic_);

    nh_.getParam("scout_plane_GPSpos_topic_",scout_plane_GPSpos_topic_);
    
    nh_.getParam("target_postion_from_image_topic_",target_postion_from_image_topic_);

    nh_.getParam("target_count_from_image_topic_",target_count_from_image_topic_);

    nh_.getParam("uav_num",uav_num);

    //相机参数读取
    nh_.param<float>("fx", fx, 268.75);
    nh_.param<float>("fy", fy, 268.75);
    nh_.param<float>("cx", cx, 320);
    nh_.param<float>("cy", cy, 240);

    //对中控制算法的比例参数
    nh_.param<double>("Kp_", Kp_, 0.1);
    nh_.param<double>("Ki", Ki_, 0.1);

    //对中最大距离阈值
    nh_.param<float>("Thres_distance_land", Thres_distance_land, 0.7);

    detectDrone_ = new AeroDrone(1);

    //无人机解锁
    if(!detectDrone_->arm())
    {
        ROS_INFO("Vehicle cann't arm...");
    }
    else
    {
        if (!detectDrone_->setMode("OFFBOARD"))
        {
            ROS_ERROR("Fatal: Set OFFBOARD mode failed!");
        }
        else
        {
            ROS_INFO("Set OFFBOARD mode sent");
        }
    }
    //起飞到8m
    detectDrone_->moveBody(0, 0, 12);
    sleep(5);

    current_status_ = my_Status_n::SEARCH;
    // --TOPICS -- //
    // 订阅无人机目前的GPS信息
    scout_curr_GPSPOS_sub_     = nh.subscribe(scout_plane_GPSpos_topic_,1,&scout_plane::GPSgetCallback,this);
    //订阅检测节点找到的无人机信息
    
    target_pos_info_sub_           = nh.subscribe("/darknet_ros/bounding_boxes",1,&scout_plane::target_pos_infocallback,this);
    target_count_info_sub_           = nh.subscribe("/darknet_ros/found_object",1,&scout_plane::target_count_infocallback,this);
    
    //订阅攻击机的状态和cost值,一份无人机的操作
    attacker_exec_result_sub_  = nh.subscribe("attacker_cost",10,&scout_plane::attackerresultinfocallback,this);
   
    //发布目标GPS信息
    target_GPS_pos_pub_        = nh.advertise<target_landing::scout_plan_msg>(target_GPS_topic_,1);
    //发布所需的无人机编号(竞标之后)
    target_dronenum_pub_       = nh.advertise<target_landing::scout_plan_msg>(target_Dronenum_topic_,1);

}

scout_plane::~scout_plane()
{
    ROS_INFO("Killing scout...");
}

//发布线程入口
void scout_plane::mission_Thread(){

    ros::Rate rate(1);
    while(ros::ok()){
        rate.sleep();
        mission_make();
    }

}

//发布线程主函数
void scout_plane::mission_make(){

    // cout << "in the pub thread"<<endl;

    if(single_distocen_ < Thres_distance_land && err_x_ <= 0.3 && err_x_ >= 0){
        
        reach_flag_ = true;

        setPUBGPSconfig();        
    }
    pubGPStime_ = ros::Time::now().toSec();
    dt_ = current_time_ - pubGPStime_;
    
    // cout << "dt_ :" << dt_<< endl;
    // cout << "cost_result_.size()::::"<< cost_result_.size()<<endl;

    //过十秒钟取这时的每个无人机的cost,并排序发布
    if(abs(dt_) > 8 && abs(dt_) < 9.5){
        
        cost_sort();//每个无人机的cost_result入vec然后进行排序,取最小的几个无人机编号
        
        setPUBGNUMconfig();//发布无人机编号
    }

}

void scout_plane::cost_sort(){

    if(cost_result_.empty())
    {
        return;
    }

    vector<pair<int, float>> vec(cost_result_.begin(), cost_result_.end());

    //对线性的vector进行排序
	sort(vec.begin(), vec.end(), cmp);//cmp不能少,cmp为自定义的比较函数

    // cout << ">>>>>>>>>>>>>>>>>>>>>>>>the_value_of_the_num_:  "<< the_value_of_the_num_<<"   "<<endl;
    scout_message_.UAV_NUM.resize(the_value_of_the_num_);
    // cout << ">>>>>>>>>>resize finish <<<<<<<<<<<<"<<endl;
    
    //对应的前数字个小的放入数组
    for (size_t i = 0; i < the_value_of_the_num_; i++)
    {
        //发布无人机编号信息
        // cout << "vec[" << i <<"].first: "<< vec[i].first <<endl;
        // cout <<" type of scout_message_.UAV_NUM[i]:" << typeid(scout_message_.UAV_NUM[i]).name() <<endl;
        scout_message_.UAV_NUM[i] = vec[i].first;
    }
}

void scout_plane::setsearchConfig()
{   
    // for(;start_vx_ < 1.2; start_vx_= start_vx_ + 0.001){
    //     vx_ = start_vx_;
    // }
    
    double sea_fl= abs(last_err_x_) - abs(err_x_);
    if(abs(sea_fl) > 0.5)
    {
        if(sea_fl > 0)
        {
            last_err_x_ = last_err_x_ - 0.02;
        }
        else
        {
            last_err_x_ = last_err_x_ + 0.02;
        }
        
    }
    vx_ = 0.1 * abs(last_err_x_);
    // vx_ = 0.1 * (abs(last_err_x_) + 0.4);
    vx_ = vx_ > 1 ? 1 : vx_;

    cmd_vel_.linear.x = vx_;
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.z = 0;
    cout << "-----------in search statue---------"<< endl;
    cout << "cmd_vel_.linear.x: "<< cmd_vel_.linear.x <<endl;
    cout << "cmd_vel_.linear.y: " << cmd_vel_.linear.y<<endl;
    
    detectDrone_->setVelocityBody(cmd_vel_.linear.x,cmd_vel_.linear.y,cmd_vel_.linear.z,cmd_vel_.angular.z);

    // cout << "----------------see_you: " << see_you <<"---------------------"<<endl; 
    //看到目标切换状态
    if(see_you){
        
        current_status_ = my_Status_n::TOCENTER; 
    }
    else
    {
        current_status_ = my_Status_n::SEARCH;
    }

    // last_err_x_ = err_x_;
    last_vx_ = cmd_vel_.linear.x;
    tocenter_vx_ = last_vx_;
    tocenter_vy_ = 0.0;
    
};

void scout_plane::setTOCENTERconfig(){    
    
    see_count_ = 0;
    // vx_ = 0;
    // start_vx_ = 0;
    // cout << " ---------------------in tocenter statue-------------------- "<<endl;
    //获得和中心坐标的差值,作为pid的输入
    
    //速度突变引起的??????
    //抖动是因为,首先在search状态,当突然看到第二个目标时,由于err_x的突变导致,速度突变(已经解决在search状态中),
    //这时由于在看到第二个目标的时候将see_you设置成了true,成了tocenter状态,但是突然看不到了下一个目标,就对准上一个目标,速度又突变了
    //目前的问题是增加了see_count_计数,但是到tocenter状态无人机速度又突变了
    
    double f_or_l_x = abs(tocenter_vx_)- abs(Kp_ * err_x_);//这里速度值太小了,设成int太傻了
    double f_or_l_y = abs(tocenter_vy_)- abs(Kp_ * err_y_);

    if(abs(f_or_l_x) > 0.15)
    {
        if(f_or_l_x > 0){
            
            tocenter_vx_ = tocenter_vx_ - 0.1;
        }
        else
        {
            tocenter_vx_ = tocenter_vx_ + 0.1;
        }     
    }
    else
    {
        tocenter_vx_ = Kp_ * err_x_;
    }
    
    if(abs(f_or_l_y) > 0.15)
    {
        if(err_y_ > 0){
            
            tocenter_vy_ = tocenter_vy_ + 0.01;
        }
        else
        {
            tocenter_vy_ = tocenter_vy_ - 0.01;
        }     
    }
    else
    {
        tocenter_vy_ = Kp_ * err_y_;
    }

    cmd_vel_.linear.x = tocenter_vx_;
    cmd_vel_.linear.y = tocenter_vy_;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.z = 0;

     cout << "-----------in center statue---------"<< endl;
     cout << "f_or_l_x: "<<f_or_l_x<<endl;
     cout << "f_or_l_y: "<<f_or_l_y<<endl;
     cout << "last_vx_: "<<last_vx_<< endl;
     cout << "abs(Kp_ * err_x_): "<< abs(Kp_ * err_x_)<<endl;
     cout << "tocenter_vx_:"<< tocenter_vx_<< endl; 
     cout << "tocenter_vy_:"<< tocenter_vy_<< endl; 
    // cout << "cmd_vel_.linear.x: " << cmd_vel_.linear.x<<endl;
    // cout << "cmd_vel_.linear.y: " << cmd_vel_.linear.y<<endl;
  
   //先将Y对准到正负0.5之间
    if(abs(err_y_) > 0.5){
        detectDrone_->setVelocityBody(0,cmd_vel_.linear.y,cmd_vel_.linear.z,cmd_vel_.angular.z);
        return;
    }
    
    //发布无人机速度
    detectDrone_->setVelocityBody(cmd_vel_.linear.x,cmd_vel_.linear.y,cmd_vel_.linear.z,cmd_vel_.angular.z);

    //阈值小于设定阈值切换状态
    if(single_distocen_ < Thres_distance_land && err_x_ <= 0.3 && err_x_ >= 0){
        
        current_status_ = my_Status_n::SEARCH;
        see_you = false;

        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << ">>>>>>>>>>>>>>>>>>>>>have been center<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    }

    last_err_x_ = err_x_;
}

void scout_plane::setPUBGPSconfig(){

    //发布目标GPS信息
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>> start pubGPS <<<<<<<<<<<<<<<<<<<<<<<"<<endl;
    scout_message_.target_GPS = scout_plane_GPS;
    target_GPS_pos_pub_.publish(scout_message_);
    
}

void scout_plane::setPUBGNUMconfig(){
    
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>> start pubNUM <<<<<<<<<<<<<<<<<<<<<<<"<<endl;

    for(size_t i = 0;i < scout_message_.UAV_NUM.size();i++)
    {
        cout << "scout_message_.UAV_NUM[" << i << "] :" <<scout_message_.UAV_NUM[i] << endl;
    }
    cout << "scout_message_.UAV_NUM.size(): "<< scout_message_.UAV_NUM.size() << endl;
    target_dronenum_pub_.publish(scout_message_);
    
    //pub 无人机编号之后 清楚cost_map(可以考虑清楚已经发布的无人机编号)
    
    cost_result_.erase(cost_result_.begin(),cost_result_.end());
    
}


void scout_plane::GPSgetCallback(const sensor_msgs::NavSatFixConstPtr& GPSpos_msg){
    
    sensor_msgs::NavSatFix GPS; 
    GPS.header.frame_id = "scout_plane";
    GPS.latitude = GPSpos_msg->latitude;
    GPS.longitude = GPSpos_msg->longitude;
    GPS.altitude = GPSpos_msg->altitude;
    if(reach_flag_ = true)
    {
       scout_plane_GPS = GPS;
    }
}

//计算目标点的Y值
double scout_plane::cal_disto_imgcenter(const double & xmax,const double & xmin,const double & ymax,const double & ymin){

    cv::Point2d point;
    point.x = (xmax + xmin) /2;    
    point.y = (ymax + ymin)/2;
    return point.y;

}


//从图像坐标系转到无人机机体坐标系
//无人机坐标系            图像坐标系
//               ^ x    |-------------> x
//               |      |    
//               |      |
// y             |      |    
//<---------------      V y
cv::Point2d scout_plane::fromcam_to_uav(const darknet_ros_msgs::BoundingBox &msg, double uav_crr_height_){

    cv::Point2d centerpoint;
    double centerx = (msg.xmax + msg.xmin) /2.0;
    double centery = (msg.ymax + msg.ymin) /2.0;

    centerpoint.x = -(centery - cy) * uav_crr_height_/ fy;
    centerpoint.y = -(centerx - cx) * uav_crr_height_/ fx;

    return centerpoint;
}

void scout_plane::target_pos_infocallback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg){
    
    // cout << "in target pos CB"<<endl;
    //先去-----无人机图像中心近的靶标
    //计算-----中心的距离,发布线程订阅到达中心就把次GPS信息发出
    //到了该靶标发送GPS信息
    //接着往前走
    //反复即可

    target_count_= msg->bounding_boxes.size();//检测出物体的数目,不需要再由ObjectCount回调获得

    // cout << "target_count_: "<<target_count_<< endl;
    
    uav_crr_height_ = detectDrone_->localPosition().pose.position.z;

    if(target_count_ == 0){ //如果没有检测到就直接下次回调
        
        cout<<" ===========No Target Found============ "<<endl;
        see_you = false;
        return;        
    }
    else if (target_count_ == 1)
    {
        // cout << "Only one target found>>>>>>>>>>>>>>move move move<<<<<<<<<<<<<<<<"<<endl;
    }
    
    //target_count >1
    vector<cv::Point2d> tpos_in_uav_vec;
    vector<double> distouav;
    tpos_in_uav_vec.resize(target_count_);
    distouav.resize(target_count_);
    
    dis_tocenter_.resize(target_count_); //对用来存每个检测到目标的vector重新规划大小

    cout << "target_count: "<< target_count_<< endl;
    
    for(int i = 0; i < target_count_; i++){//计算出每一个目标点的Y值,企图找到Y值最大的点,也就是从图像下方看过去最近的点
        
        //每个目标在无人机坐标系下的位置
        tpos_in_uav_vec[i] = fromcam_to_uav(msg->bounding_boxes[i],uav_crr_height_);
        cout << "tpos_in_uav_vec["<< i <<"] :" <<tpos_in_uav_vec[i] << endl;
        distouav[i] = sqrt(tpos_in_uav_vec[i].x * tpos_in_uav_vec[i].x + tpos_in_uav_vec[i].y * tpos_in_uav_vec[i].y);
        
        //这个阈值0.2要小于,对中判断的err_x阈值,一旦这边满足条件小于阈值,切换目标之后,那边err_x_虽然也满足了条件,
        //但是distance_tocenter就是切换之后的了
        if(tpos_in_uav_vec[i].x > 0.2)//只有目标在无人机前面才会计入排序,或距离大于阈值
        {     
            //记录在无人机前面的目标的中心Y值
            dis_tocenter_[i] = cal_disto_imgcenter(msg->bounding_boxes[i].xmax,msg->bounding_boxes[i].xmin,msg->bounding_boxes[i].ymax,msg->bounding_boxes[i].ymin);  
        }
        else
        {
            dis_tocenter_[i] = 0.0;
        }

        // cout << "dis_tocenter_["<< i <<"] :" << dis_tocenter_[i] << endl;
        
    }

    //最大值对应的迭代器
    auto maxest = max_element(dis_tocenter_.begin(),dis_tocenter_.end());
    //最大值下标
    int maxindex = distance(begin(dis_tocenter_),maxest);
    
    int targetindex = maxindex;
    cout << "targetindex: "<< targetindex << endl;
    // cout <<"tpos_in_uav_vec[targetindex].x: "<<tpos_in_uav_vec[targetindex].x<<endl;
    if(tpos_in_uav_vec[targetindex].x > 1){
        
        see_count_++;
        // cout << "see_count: "<< see_count_ << endl;
        if(see_count_ > 30){

            cout << "target is valid, see_you set to true."<<endl;
            //设定找到目标点flag
            see_you = true;
        }

    }
    
    scout_message_.num_val = msg->bounding_boxes[targetindex].id + 1;//识别到的数字大小等于识别的id加一(darknet_ros的设置)
    the_value_of_the_num_ = scout_message_.num_val;
    cout << "the_value_of_the_num_: "<< the_value_of_the_num_<< endl;

    // double centerx = (msg->bounding_boxes[targetindex].xmax + msg->bounding_boxes[targetindex].xmin) /2.0;//最近点的x坐标
    // double centery = (msg->bounding_boxes[targetindex].ymax + msg->bounding_boxes[targetindex].ymin) /2.0;//最近点的y坐标
    
    // cout << "centerx :"<<centerx<<endl;
    // cout << "centery :"<<centery<<endl;
    
    //下一个目标点,和中心点的误差
    //对应和中心点的差值:    无人机坐标系下的误差值,速度按照此误差来计算:      
    //centerx - cx        -(centerx - cx) / fx
    //centery - cy        -(centery - cy) / fy

    cout<< "target_id:" << msg->bounding_boxes[targetindex].id + 1<<endl;
    
    //这里对应的x,y坐标系是反过来的,是最终无人机的对应的误差值
    // err_x_ = -(centery - cy) * uav_crr_height_/ fy;
    // err_y_ = -(centerx - cx) * uav_crr_height_/ fx;

    err_x_ = tpos_in_uav_vec[targetindex].x;
    err_y_ = tpos_in_uav_vec[targetindex].y;

    //cout << "int image CB , see_you:"<< see_you <<"-------"<<endl;
    
    //计算到目标中心点的距离
    single_distocen_ =  distouav[targetindex];
    cout << "err_x_: "<< err_x_ <<endl;
    cout << "err_y_: "<< err_y_ <<endl;

    //  cout << "detectDrone_->pitchAngle() :"<< detectDrone_->pitchAngle() << endl;
    //  cout << "detectDrone_->pitchRad() :"<< detectDrone_->pitchRad() << endl;

    if(current_status_ == my_Status_n::TOCENTER){

        cout << "single_distocen_: "<< single_distocen_ <<endl;
    }

}

void scout_plane::target_count_infocallback(const darknet_ros_msgs::ObjectCountConstPtr &msg){
    
    //target_count_ = msg->count;
    // cout << "msg->count::::"<< msg->count <<endl;

}

void scout_plane::attackerresultinfocallback(const target_landing::AttackerMsgConstPtr& msg){
    
    cout << "attacker cost cB"<<endl;
    //记录每次获得cost的时间
    current_time_ = ros::Time::now().toSec();
    //来一个就压进去,key是不能重复的,每次循环用完结束需要清空一下
    cost_result_.insert(std::make_pair(msg->UAV_NUM,msg->cost));
   
}

//整个流程中整个状态切换的函数
void scout_plane::run(){   
    /*
        calculate the min cost and the bind num, and set attacker's status
    */
   switch (current_status_)
   {
        case my_Status_n::SEARCH:
            setsearchConfig();
            break;
        case my_Status_n::TOCENTER:
            setTOCENTERconfig();
            break;
        default:
            break;
   }
}


int main(int argc, char **argv){
    
    ros::init(argc, argv, "scout_plane");
    ros::NodeHandle nh_("~");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    scout_plane scouter(nh,nh_);

    thread pubthread(&scout_plane::mission_Thread,&scouter); 
    
    while (ros::ok())
    {
        scouter.run();
        ros::spinOnce();
        rate.sleep();
    }
    
    pubthread.join();

    return 0;
}
