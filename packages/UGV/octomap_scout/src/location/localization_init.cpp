
 /*
 * localization_init.cpp
 * 
 * Author： Born Chow
 * Date: 2020.07.14
 * 
 * 说明: 使用ndt进行全局地图上的初始位置匹配
 * 参考: https://blog.csdn.net/adamshan/article/details/79230612
 * 速度慢，不知原因
 */
# include <ros/ros.h>
# include <tf/transform_listener.h>
# include <geometry_msgs/PointStamped.h>
# include <sensor_msgs/PointCloud2.h>
# include <geometry_msgs/PoseWithCovarianceStamped.h>

# include <iostream>
# include <mutex>

# include <pcl_ros/point_cloud.h>
# include <pcl_ros/transforms.h>
# include <pcl_ros/impl/transforms.hpp>
# include <pcl_conversions/pcl_conversions.h>

# include <pcl/point_types.h>
# include <pcl/registration/ndt.h>
# include <pcl/filters/voxel_grid.h>

# include <tf2/transform_datatypes.h>
# include <tf2_geometry_msgs/tf2_geometry_msgs.h>
# include <tf2_ros/transform_broadcaster.h>
# include <tf2_eigen/tf2_eigen.h>
# include <tf2_ros/transform_listener.h>

# include "tic_toc.hpp"

using namespace std;
typedef pcl::PointXYZ PCLPoint;

class Localization_init
{
private:
    ros::NodeHandle n_;

    ros::Subscriber global_map_sub_;
    ros::Subscriber scan_pc_sub_;
    ros::Subscriber initial_pose_sub_;

    ros::Publisher ndt_pose_pub_;
    ros::Publisher sensor_aligned_pose_pub_;

    pcl::PointCloud<PCLPoint>::Ptr global_pc_ptr_;
    pcl::PointCloud<PCLPoint>::Ptr scan_pc_ptr_;

    //ndt param
    float res_;
    float step_size_;
    float trans_eps_;
    float max_iter_;
    pcl::NormalDistributionsTransform<PCLPoint, PCLPoint>::Ptr ndt_ptr_;
    double converged_param_transform_probability_;

    // filter param
    pcl::VoxelGrid<PCLPoint> voxel_filter_;
    float leaf_size_x_, leaf_size_y_, leaf_size_z_;

    //位姿相关
    bool init_pose_;
    bool global_pc_init_;
    // init guess for ndt
    geometry_msgs::PoseWithCovarianceStamped initial_pose_cov_msg_;
    string map_frame_;
    Eigen::Matrix4f pre_trans, delta_trans;

    double sensor_ros_time_;
    //话题
    string global_map_topic_, scan_pc_topic_;

    std::mutex ndt_map_mtx_;


private:
    void global_map_cb(const sensor_msgs::PointCloud2ConstPtr &msg);

    void scan_pc_cb(const sensor_msgs::PointCloud2ConstPtr &msg);

    void callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_conv_msg_ptr);

    bool setRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

    void filterPointCloud(pcl::PointCloud<PCLPoint>::Ptr &input_pc_ptr, pcl::PointCloud<PCLPoint>::Ptr &filterd_pc_ptr);
    bool setFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    void matchScan();

    void init_params(ros::NodeHandle &nh_);


public:
    Localization_init(ros::NodeHandle& nh, ros::NodeHandle &private_nh);
    ~Localization_init();
};

Localization_init::Localization_init(ros::NodeHandle& nh, ros::NodeHandle &private_nh) :
n_(nh),
global_pc_ptr_(new pcl::PointCloud<PCLPoint>),
scan_pc_ptr_(new pcl::PointCloud<PCLPoint>),
ndt_ptr_(new pcl::NormalDistributionsTransform<PCLPoint, PCLPoint>())
{
    init_params(private_nh);

    global_map_sub_ = n_.subscribe<sensor_msgs::PointCloud2>(global_map_topic_, 10, &Localization_init::global_map_cb, this);
    scan_pc_sub_ = n_.subscribe<sensor_msgs::PointCloud2>(scan_pc_topic_, 10 ,&Localization_init::scan_pc_cb, this);
    initial_pose_sub_ = n_.subscribe("initialpose", 100, &Localization_init::callback_init_pose, this);

    ndt_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);
    sensor_aligned_pose_pub_ = n_.advertise<sensor_msgs::PointCloud2>("points_aligned", 10);

}

Localization_init::~Localization_init()
{
}

void Localization_init::init_params(ros::NodeHandle &nh_){
    //需要初始化的参数
    //从ros 参数服务器中获取
    nh_.param<string>("global_map_topic", global_map_topic_, "/points_map");
    nh_.param<string>("scan_pc_topic", scan_pc_topic_, "/velodyne_points");

    // ndt参数
    nh_.param<float>("resolution", res_, 2.0);
    nh_.param<float>("step_size", step_size_, 0.1);
    nh_.param<float>("trans_epsilon", trans_eps_, 0.05);
    nh_.param<float>("max_iterations", max_iter_, 30.0);
    nh_.param<double>("converged_param_transform_probability", converged_param_transform_probability_, 3.0);

    //voxel filter param
    nh_.param<float>("leaf_size_x", leaf_size_x_, 1.0);
    nh_.param<float>("leaf_size_y", leaf_size_y_, 1.0);
    nh_.param<float>("leaf_size_z", leaf_size_z_, 1.0);

    map_frame_ = "map";
    init_pose_ = false;
    global_pc_init_ = false;

    //初始化匹配器与滤波器
    setRegistrationParam(res_, step_size_, trans_eps_, max_iter_);
    setFilterParam(leaf_size_x_, leaf_size_y_, leaf_size_z_);

    cout<< "-----------param init finished----------------------" << endl;

}

// void Localization_init::global_map_cb(const sensor_msgs::PointCloud2ConstPtr &msg){
//     if(!global_pc_init_){
//         pcl::fromROSMsg(*msg, *global_pc_ptr_);
//         ndt_ptr_->setInputTarget(global_pc_ptr_);
//         global_pc_init_ = true;
//     }

// }

void Localization_init::global_map_cb(const sensor_msgs::PointCloud2::ConstPtr & msg){
    if(!global_pc_init_){
        const auto trans_epsilon = ndt_ptr_->getTransformationEpsilon();
        const auto step_size = ndt_ptr_->getStepSize();
        const auto resolution = ndt_ptr_->getResolution();
        const auto max_iterations = ndt_ptr_->getMaximumIterations();

        pcl::NormalDistributionsTransform<PCLPoint, PCLPoint> ndt_new;

        ndt_new.setTransformationEpsilon(trans_epsilon);
        ndt_new.setStepSize(step_size);
        ndt_new.setResolution(resolution);
        ndt_new.setMaximumIterations(max_iterations);

        pcl::PointCloud<PCLPoint>::Ptr map_points_ptr(new pcl::PointCloud<PCLPoint>);
        pcl::fromROSMsg(*msg, *map_points_ptr);
        ndt_new.setInputTarget(map_points_ptr);
        // create Thread
        // detach
        pcl::PointCloud<PCLPoint>::Ptr output_cloud(new pcl::PointCloud<PCLPoint>);
        ndt_new.align(*output_cloud, Eigen::Matrix4f::Identity());

        //   swap
        ndt_map_mtx_.lock();
        *ndt_ptr_ = ndt_new;
        ndt_map_mtx_.unlock();
        global_pc_init_ = true;

    }

}

void Localization_init::scan_pc_cb(const sensor_msgs::PointCloud2ConstPtr &msg){
    pcl::fromROSMsg(*msg, *scan_pc_ptr_);
    sensor_ros_time_ = msg->header.stamp.toSec();
    matchScan();
}


void Localization_init::callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & initial_pose_msg_ptr){
  if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
    initial_pose_cov_msg_ = *initial_pose_msg_ptr;
  } else {
      ;
  }
  // if click the initpose again, re init
  init_pose_ = false;
}

bool Localization_init::setRegistrationParam(float res, float step_size, float trans_eps, int max_iter){
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);

    std::cout << "NDT 的匹配参数为：" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool Localization_init::setFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z){
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    std::cout << "Voxel Filter 的参数为：" << std::endl
            << leaf_size_x << ", "
            << leaf_size_y << ", "
            << leaf_size_z 
            << std::endl << std::endl;
    return true;
}

void Localization_init::filterPointCloud(pcl::PointCloud<PCLPoint>::Ptr &input_pc_ptr, pcl::PointCloud<PCLPoint>::Ptr &filterd_pc_ptr){
    voxel_filter_.setInputCloud(input_pc_ptr);
    voxel_filter_.filter(*filterd_pc_ptr);
}

void Localization_init::matchScan(){

    TicToc exe_time_cnt;
    // mutex Map
    std::lock_guard<std::mutex> lock(ndt_map_mtx_);

    pcl::PointCloud<PCLPoint>::Ptr scan_pc_ds_ptr (new pcl::PointCloud<PCLPoint>);
    filterPointCloud(scan_pc_ptr_, scan_pc_ds_ptr);
    cout << "scan_pc_ds size: " << scan_pc_ds_ptr->size() <<endl;
    ndt_ptr_->setInputSource(scan_pc_ds_ptr);
    if (ndt_ptr_->getInputTarget() == nullptr) {
        ROS_WARN_STREAM_THROTTLE(1, "No MAP!");
        return;
    }
    // 设置初始位姿 
    Eigen::Matrix4f initial_pose_matrix;
    if (!init_pose_){
        Eigen::Affine3d initial_pose_affine;
        tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
        initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
        // for the first time, we don't know the pre_trans, so just use the init_trans, 
        // which means, the delta trans for the second time is 0
        pre_trans = initial_pose_matrix;
        init_pose_ = true;
    }else
    {
        // use predicted pose as init guess (currently we only impl linear model)
        initial_pose_matrix = pre_trans * delta_trans;
    }

    TicToc align_time_cnt;
    pcl::PointCloud<PCLPoint>::Ptr scan_pc_output_ptr (new pcl::PointCloud<PCLPoint>);
    ndt_ptr_->align(*scan_pc_output_ptr, initial_pose_matrix);
    double align_time = align_time_cnt.toc();

    const Eigen::Matrix4f result_pose_matrix = ndt_ptr_->getFinalTransformation();
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

    const float transform_probability = ndt_ptr_->getTransformationProbability();
    const int iteration_num = ndt_ptr_->getFinalNumIteration();

    bool is_converged = true;
    static size_t skipping_publish_num = 0;
    if (
        iteration_num >= ndt_ptr_->getMaximumIterations() + 2 ||
        transform_probability < converged_param_transform_probability_) {
        is_converged = false;
        ++skipping_publish_num;
        std::cout << "Not Converged" << std::endl;
    } else {
        skipping_publish_num = 0;
    }

    //计算当前帧与上一帧的变换量
    // pre_trans * delta_trans = result_pose_matrix
    // 上一帧　×　delta = 当前帧
    delta_trans = pre_trans.inverse() * result_pose_matrix;
    // 将当前帧赋值给上一帧
    pre_trans = result_pose_matrix;

    Eigen::Vector3f delta_translation = delta_trans.block<3, 1>(0, 3);
    cout<<"delta x: "<<delta_translation(0) << " y: "<<delta_translation(1)<<
            " z: "<<delta_translation(2)<<endl;

    Eigen::Matrix3f delta_rotation_matrix = delta_trans.block<3, 3>(0, 0);
    Eigen::Vector3f delta_euler = delta_rotation_matrix.eulerAngles(2,1,0);
    cout<<"delta yaw: "<<delta_euler(0) << " pitch: "<<delta_euler(1)<<
            " roll: "<<delta_euler(2)<<endl;

    // publish
    geometry_msgs::PoseStamped result_pose_stamped_msg;
    result_pose_stamped_msg.header.stamp = ros::Time().fromSec(sensor_ros_time_);
    result_pose_stamped_msg.header.frame_id = map_frame_;
    result_pose_stamped_msg.pose = result_pose_msg;

    if (is_converged) {
        ndt_pose_pub_.publish(result_pose_stamped_msg);
    }

    // publish aligned point cloud
    pcl::PointCloud<PCLPoint>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<PCLPoint>);
    pcl::transformPointCloud(*scan_pc_ds_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
    sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
    pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
    sensor_points_mapTF_msg.header.stamp = ros::Time().fromSec(sensor_ros_time_);
    sensor_points_mapTF_msg.header.frame_id = map_frame_;
    sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);

    double exe_time = exe_time_cnt.toc();

    std::cout << "align_time: " << align_time << "ms" << std::endl;
    std::cout << "exe_time: " << exe_time << "ms" << std::endl;
    std::cout << "trans_prob: " << transform_probability << std::endl;
    std::cout << "iter_num: " << iteration_num << std::endl;
    std::cout << "skipping_publish_num: " << skipping_publish_num << std::endl;
    std::cout << "------------------------------------------------" << std::endl;
}

// class Localization_init end;
//-----------------------------------------------------------------------------------------------//

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_node");
    ROS_INFO("\033[1;32m---->\033[0m Localization Node Started.");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate rate(30.0);

    Localization_init localizer(nh, nh_);

    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    ros::spin();
    return 0;

}










