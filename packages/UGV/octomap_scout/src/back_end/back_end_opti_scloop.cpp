 /*
 * back_end_opti.cpp
 * 
 * Author： Born Chow
 * Date: 2020.07.23
 * 
 * 说明: 对aloam进行后端优化和回环检测
 * 参考:Lego_loam aloam
 * 【订阅】
 */

// ros相关
# include <ros/ros.h>
# include <tf/transform_listener.h>
# include <nav_msgs/Odometry.h>
# include <geometry_msgs/PointStamped.h>
# include <sensor_msgs/PointCloud2.h>
# include <geometry_msgs/PoseWithCovarianceStamped.h>
# include "visualization_msgs/Marker.h"

# include <pcl/point_cloud.h>
# include <pcl/point_types.h>
# include <pcl/filters/voxel_grid.h>
# include <pcl/common/transforms.h>	 //pcl::transformPointCloud() in this
# include <pcl/kdtree/kdtree_flann.h> // pcl::KdTreeFLANN in this
# include <pcl/registration/icp.h> //pcl::IterativeClosestPoint<PointType, PointType> in this
# include <pcl/io/pcd_io.h>

# include <pcl_conversions/pcl_conversions.h>

# include "tictoc.h"
# include "Scancontext.h"

# include <mutex>
# include <queue>
# include <thread>

# include <gtsam/geometry/Rot3.h>
# include <gtsam/geometry/Pose3.h>
# include <gtsam/slam/PriorFactor.h>
# include <gtsam/slam/BetweenFactor.h>
# include <gtsam/nonlinear/NonlinearFactorGraph.h>
# include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
# include <gtsam/nonlinear/Marginals.h>
# include <gtsam/nonlinear/Values.h>
# include <gtsam/nonlinear/ISAM2.h>

typedef pcl::PointXYZI  PointType;
using namespace std;
using namespace gtsam;

/*
* A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
*/
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

// class 
class BackEndOpti
{
private:
    ros::NodeHandle n_;

    ros::Subscriber scan_pc_sub_;
    ros::Subscriber laser_odom_sub_;

    ros::Publisher key_pose_pub_;
    ros::Publisher key_frame_pub_;
    ros::Publisher global_map_pub_;
    ros::Publisher history_frame_pub_;
    ros::Publisher loop_line_pub_;

    // gtsam相关
    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;
    gtsam::ISAM2 *isam;
    gtsam::Values isamCurrentEstimate;
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise;
    gtsam::noiseModel::Base::shared_ptr robustNoiseModel;

    std::queue<sensor_msgs::PointCloud2ConstPtr> scan_pc_buf_;
    std::queue<nav_msgs::Odometry::ConstPtr> laser_odom_buf_;
    std::mutex mBuf;
    std::mutex mtx;
    // std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
    // std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;   //关键帧3D位态集合
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;  //关键帧6D位态集合
    vector<pcl::PointCloud<PointType>::Ptr> pc_key_frames_;  //关键帧点云集合

    pcl::PointCloud<PointType>::Ptr scan_pc_ptr_; //当前帧点云 

    PointType cloudPose3D; //当前帧3D位态
    PointTypePose cloudPose6D; //当前帧6D位态

    PointType lastKeyPose3D; //上一帧关键帧位置
    PointTypePose lastKeyPose6D;

    
    // 回环检测
    // kt-tree
    // 用于回环检测的位置kt-tree
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;
    bool aLoopIsClosed; //是否回环标志
    
    int historyKeyframeSearchNum = 25; // 回环关键帧左右n帧构建为icp的target点云 25
    float historyKeyframeFitnessScore = 1.5; //icp迭代中的分数阈值

    // RS回环 RS: radius search 
    float historyKeyframeSearchRadius = 10.0; // RS回环距离 20.0
    int RSclosestHistoryFrameID;  // 回环关键帧id, RS回环中，当前帧与哪一关键帧构成回环
    int latestFrameIDLoopCloure; // 【当前】关键帧id

    visualization_msgs::Marker line; //可视化回环关系

    pcl::PointCloud<PointType>::Ptr RSlatestSurfKeyFrameCloud; // 【当前】关键帧id对应点云
    pcl::PointCloud<PointType>::Ptr RSnearHistorySurfKeyFrameCloud; // 回环关键帧id【附近】对应点云
    pcl::PointCloud<PointType>::Ptr RSnearHistorySurfKeyFrameCloudDS;
    //SC 回环
    pcl::PointCloud<PointType>::Ptr SClatestSurfKeyFrameCloud; // 【当前】关键帧id对应点云
    pcl::PointCloud<PointType>::Ptr SCnearHistorySurfKeyFrameCloud; // 回环关键帧id【附近】对应点云
    pcl::PointCloud<PointType>::Ptr SCnearHistorySurfKeyFrameCloudDS;
    int SCclosestHistoryFrameID;
    // // loop detector 
    SCManager scManager;
    float yawDiffRad;
    
    //降采样
    pcl::VoxelGrid<PointType> downSizeFilterHistoryKeyFrames;  //降采样 <回环关键帧id【附近】对应点云>
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // 降采样全局地图
    pcl::VoxelGrid<PointType> downSizeFilterKeyFrames; // 降采样关键帧

    // 时间戳
    double time_scan_pc;
    double time_laser_odom;

    // 保存点云路径
    std::string pcd_file_path;

    // 发布回环的地图
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap; // kt tree 用于搜索当前帧附近的关键帧pose，
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses; // 搜索出来的关键帧pose
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames; // 搜索出来的关键帧点云(所有关键帧加起来－>相当于全局地图)
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;
    float globalMapVisualizationSearchRadius = 1500.0; // 发布的全局地图的范围


    
    
private:
    void scanPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void laserOdomCallback(const nav_msgs::Odometry::ConstPtr &laser_odom_msg);
    bool timeSyncData();
    // void pcTransToInit(pcl::PointCloud<PointType>::Ptr &pc, PointTypePose pose);
    pcl::PointCloud<PointType>::Ptr pcTransToInit(pcl::PointCloud<PointType>::Ptr pc, PointTypePose pose);
    Eigen::Affine3f pclPointToAffine3f(PointTypePose pose);
    void init_params(ros::NodeHandle &nh_);
    bool insertKeyFrame();
    void correctPoses();
    void publishKeyPosesAndFrames();
    
public:
    BackEndOpti(ros::NodeHandle& nh, ros::NodeHandle &private_nh);
    ~BackEndOpti();
    void run();
    void performLoopClosure();
    bool detectLoopClosure();
    void publishGlobalMap();

    void loopClosureThread();
    void visualizeGlobalMapThread();


};  // class 申明

BackEndOpti::BackEndOpti(ros::NodeHandle& nh, ros::NodeHandle &private_nh) :
n_(nh)
{
    init_params(private_nh);
    scan_pc_sub_ = n_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &BackEndOpti::scanPcCallback, this);
    laser_odom_sub_ = n_.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 10, &BackEndOpti::laserOdomCallback, this);

    key_pose_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
    key_frame_pub_ = n_.advertise<sensor_msgs::PointCloud2>("current_key_frame_mapTF", 2);
    global_map_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map_looped", 2);
    history_frame_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surronding_looped", 2);
    loop_line_pub_ = n_.advertise<visualization_msgs::Marker>("loop_line", 10);
}

BackEndOpti::~BackEndOpti()
{
}

void BackEndOpti::init_params(ros::NodeHandle &nh_){
    cout<< "----------init params----------" << endl;

    nh_.param<std::string>("file_dir", pcd_file_path, "/home/bornchow/workfiles/pcd_data/");

    // -----------------pcl指针初始化----------------
    scan_pc_ptr_.reset(new pcl::PointCloud<PointType>);
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    //发布全局地图相关
    kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>);
    globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
    globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
    globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
    globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());
    // RS回环
    RSlatestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>()); 
    RSnearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    RSnearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

    // CS回环
    SClatestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>()); 
    SCnearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
    SCnearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

    aLoopIsClosed = false; //回环是否成功标志

    // 可视化回环关系
    line.header.frame_id="/camera_init";
    line.header.stamp=ros::Time(0);
    line.ns = "lines";
    line.id = 1;
    line.type=line.LINE_LIST;
    line.action=line.ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x =  0.03;
    line.scale.y= 0.03;
    line.color.r =255.0/255.0;
    line.color.g= 155.0/255.0;
    line.color.b =155.0/255.0;
    line.color.a = 1.0;
    line.lifetime = ros::Duration();

    // 降采样参数
    float filter_size;
    //　降采样回环关键帧附近点云
    filter_size = 0.3; downSizeFilterHistoryKeyFrames.setLeafSize(filter_size, filter_size, filter_size);
    filter_size = 0.1; downSizeFilterGlobalMapKeyFrames.setLeafSize(filter_size, filter_size, filter_size); // for global map visualization
    filter_size = 0.1; downSizeFilterKeyFrames.setLeafSize(filter_size, filter_size, filter_size);

    // ------------------init 图优化器------------------
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    // ------------------noise set设置gtsam优化噪声--------------
    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
    priorNoise = noiseModel::Diagonal::Variances(Vector6);
    odometryNoise = noiseModel::Diagonal::Variances(Vector6);

}

void BackEndOpti::scanPcCallback(const sensor_msgs::PointCloud2ConstPtr& scan_pc_msg){
    mBuf.lock();
    scan_pc_buf_.push(scan_pc_msg);
    mBuf.unlock();
}

void BackEndOpti::laserOdomCallback(const nav_msgs::Odometry::ConstPtr &laser_odom_msg){
    mBuf.lock();
    laser_odom_buf_.push(laser_odom_msg);
    mBuf.unlock();
}

// void BackEndOpti::pcTransToInit(pcl::PointCloud<PointType>::Ptr &pc, PointTypePose pose){
//     Eigen::Translation3f tf_t(pose.x, pose.y, pose.z);
//     Eigen::AngleAxisf rot_x(pose.roll, Eigen::Vector3f::UnitX());
//     Eigen::AngleAxisf rot_y(pose.pitch, Eigen::Vector3f::UnitY());
//     Eigen::AngleAxisf rot_z(pose.yaw, Eigen::Vector3f::UnitZ());
//     Eigen::Matrix4f current_to_init_matrix = (tf_t * rot_z * rot_y * rot_x).matrix();
//     pcl::transformPointCloud(*pc, *pc, current_to_init_matrix);
// }

Eigen::Affine3f BackEndOpti::pclPointToAffine3f(PointTypePose pose){
    Eigen::Translation3f tf_t(pose.x, pose.y, pose.z);
    Eigen::AngleAxisf rot_x(pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y(pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z(pose.yaw, Eigen::Vector3f::UnitZ());
    return tf_t * rot_z * rot_y * rot_x;
}

pcl::PointCloud<PointType>::Ptr BackEndOpti::pcTransToInit(pcl::PointCloud<PointType>::Ptr pc, PointTypePose pose){

    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    Eigen::Translation3f tf_t(pose.x, pose.y, pose.z);
    Eigen::AngleAxisf rot_x(pose.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y(pose.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z(pose.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f current_to_init_matrix = (tf_t * rot_z * rot_y * rot_x).matrix();
    pcl::transformPointCloud(*pc, *cloudOut, current_to_init_matrix);
    return cloudOut;
}

void BackEndOpti::publishKeyPosesAndFrames(){
    if (key_pose_pub_.getNumSubscribers() != 0){
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(time_laser_odom);
        cloudMsgTemp.header.frame_id = "/camera_init";
        key_pose_pub_.publish(cloudMsgTemp);
    }

    if (key_frame_pub_.getNumSubscribers() != 0)
    {
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::PointCloud<PointType>::Ptr cloudTemp(new pcl::PointCloud<PointType>);
        // cout << "pc_key id " << pc_key_frames_.size() -1 << endl;
        // cout << "pc_key_pose id " << cloudKeyPoses6D->points.size()-1 << endl;
        // cout << "key_pose xyz" << cloudKeyPoses6D->points[cloudKeyPoses6D->points.size()-1].x << " " 
        //                        << cloudKeyPoses6D->points[cloudKeyPoses6D->points.size()-1].y << " " 
        //                        << cloudKeyPoses6D->points[cloudKeyPoses6D->points.size()-1].z << " " << endl;
        // cout << "key_pose rpy" << cloudKeyPoses6D->points[cloudKeyPoses6D->points.size()-1].roll << " " 
        //                        << cloudKeyPoses6D->points[cloudKeyPoses6D->points.size()-1].pitch << " " 
        //                        << cloudKeyPoses6D->points[cloudKeyPoses6D->points.size()-1].yaw << " " << endl;

        cloudTemp = pcTransToInit(pc_key_frames_[pc_key_frames_.size() -1], cloudKeyPoses6D->points[cloudKeyPoses6D->points.size()-1]);
        pcl::toROSMsg(*cloudTemp, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(time_laser_odom);
        cloudMsgTemp.header.frame_id = "/camera_init";
        key_frame_pub_.publish(cloudMsgTemp);
    }

    // if (pubRecentKeyFrames.getNumSubscribers() != 0){
    //     sensor_msgs::PointCloud2 cloudMsgTemp;
    //     pcl::toROSMsg(*laserCloudSurfFromMapDS, cloudMsgTemp);
    //     cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    //     cloudMsgTemp.header.frame_id = "/camera_init";
    //     pubRecentKeyFrames.publish(cloudMsgTemp);
    // }

    // if (pubRegisteredCloud.getNumSubscribers() != 0){
    //     pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    //     PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
    //     *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
    //     *cloudOut += *transformPointCloud(laserCloudSurfTotalLast, &thisPose6D);
        
    //     sensor_msgs::PointCloud2 cloudMsgTemp;
    //     pcl::toROSMsg(*cloudOut, cloudMsgTemp);
    //     cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    //     cloudMsgTemp.header.frame_id = "/camera_init";
    //     pubRegisteredCloud.publish(cloudMsgTemp);
    // } 
}

bool BackEndOpti::timeSyncData(){
    while (!scan_pc_buf_.empty() && !laser_odom_buf_.empty()) 
    {
        mBuf.lock();
        TicToc time_sync;
        // cout << "time before" << " scan_pc " << scan_pc_buf_.front()->header.stamp.toSec() << " "
        //                       << "laser_odom " << laser_odom_buf_.front()->header.stamp.toSec() << endl;
        while (!scan_pc_buf_.empty() && scan_pc_buf_.front()->header.stamp.toSec() < laser_odom_buf_.front()->header.stamp.toSec())
            scan_pc_buf_.pop();
        
        if (scan_pc_buf_.empty())
        {
            mBuf.unlock();
            return false;
        }

        time_scan_pc = scan_pc_buf_.front()->header.stamp.toSec();
        time_laser_odom = laser_odom_buf_.front()->header.stamp.toSec();
        
        if (time_scan_pc != time_laser_odom)
        {
            cout<< "time: " << "scan_pc " << time_scan_pc << " "
                            << "laser_odom" << time_laser_odom << endl;
            cout<< "unsync messeage!" << endl;

            while(!laser_odom_buf_.empty())
            {
                laser_odom_buf_.pop();
                printf("drop lidar frame in mapping for real time performance \n");
            }
            mBuf.unlock();
            return false;
        }
        
        scan_pc_ptr_->clear();
        pcl::fromROSMsg(*scan_pc_buf_.front(), *scan_pc_ptr_);
        scan_pc_buf_.pop();


        cloudPose3D.x = laser_odom_buf_.front()->pose.pose.position.x;
        cloudPose3D.y = laser_odom_buf_.front()->pose.pose.position.y;
        cloudPose3D.z = laser_odom_buf_.front()->pose.pose.position.z;

        geometry_msgs::Quaternion geoQuat = laser_odom_buf_.front()->pose.pose.orientation;
        double roll, pitch, yaw;
        // tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw); //??? 
        tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw); //??? 

        cloudPose6D.x = laser_odom_buf_.front()->pose.pose.position.x;
        cloudPose6D.y = laser_odom_buf_.front()->pose.pose.position.y;
        cloudPose6D.z = laser_odom_buf_.front()->pose.pose.position.z;
        cloudPose6D.roll = roll;
        cloudPose6D.pitch = pitch;
        cloudPose6D.yaw = yaw;
        // cloudPose6D.roll = -pitch;
        // cloudPose6D.pitch = -yaw;
        // cloudPose6D.yaw = roll;
        cloudPose6D.time = time_laser_odom;
        laser_odom_buf_.pop();

        // 删除缓存
        while(!laser_odom_buf_.empty())
        {
            laser_odom_buf_.pop();
            printf("drop lidar frame in mapping for real time performance \n");
        }
        
        time_sync.toc("time sync");
        return true;
    }

    return false;
    
}

bool BackEndOpti::insertKeyFrame(){
    bool save_this_key_frame = false;
    TicToc time_insert_key_frame;
    if(cloudKeyPoses3D->points.empty() == true){ //说明关键帧中没有数据,该帧为第一帧

        // cloudPose3D.intensity = 0;  // intensity 存关键帧的index
        // cloudKeyPoses3D->push_back(cloudPose3D);
        
        // cloudPose6D.intensity = cloudPose3D.intensity;
        // cloudKeyPoses6D->push_back(cloudPose6D);
        // pc_key_frames_.push_back(scan_pc_ptr_);
        
        //关键帧加入因子图
        // 设置图边 --> 初始边
        gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(cloudPose6D.yaw, cloudPose6D.pitch, cloudPose6D.roll),
                                                       Point3(cloudPose6D.x, cloudPose6D.y, cloudPose6D.z)), priorNoise));
        // 设置图节点
        initialEstimate.insert(0, Pose3(Rot3::RzRyRx(cloudPose6D.yaw, cloudPose6D.pitch, cloudPose6D.roll),
                                                Point3(cloudPose6D.x, cloudPose6D.y, cloudPose6D.z)));
     
        // //更新last关键帧
        // lastKeyPose3D = cloudPose3D;
        // lastKeyPose6D = cloudPose6D;
      
        // // update isam
        // isam->update(gtSAMgraph, initialEstimate);
        // isam->update();
       
        // gtSAMgraph.resize(0);
        // initialEstimate.clear();
  
        // return true;

        save_this_key_frame = true;
    } 
    // 否则计算距离判断是否加入关键帧
    else if(sqrt((cloudPose3D.x - lastKeyPose3D.x)*(cloudPose3D.x - lastKeyPose3D.x) 
          + (cloudPose3D.y - lastKeyPose3D.y)*(cloudPose3D.y - lastKeyPose3D.y)
          + (cloudPose3D.z - lastKeyPose3D.z)*(cloudPose3D.z - lastKeyPose3D.z)) > 0.3){ //每0.3米存一个关键帧
            // 需要先加入边再更新关键帧，否则index对不上
            // 关键帧加入因子图 
            // 设置图边 -->当前帧位置与上一关键帧位置的变换
            gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(lastKeyPose6D.yaw, lastKeyPose6D.pitch, lastKeyPose6D.roll),
                                                 Point3(lastKeyPose6D.x, lastKeyPose6D.y, lastKeyPose6D.z));

            gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(cloudPose6D.yaw, cloudPose6D.pitch, cloudPose6D.roll),
                                               Point3(cloudPose6D.x, cloudPose6D.y, cloudPose6D.z));

            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->points.size()-1,cloudKeyPoses3D->points.size(), poseFrom.between(poseTo), odometryNoise));
            // 设置图节点
            initialEstimate.insert(cloudKeyPoses3D->points.size(), Pose3(Rot3::RzRyRx(cloudPose6D.yaw, cloudPose6D.pitch, cloudPose6D.roll),
                                                    Point3(cloudPose6D.x, cloudPose6D.y, cloudPose6D.z)));

            // // update isam
         
            // isam->update(gtSAMgraph, initialEstimate);
            // isam->update();
     
            // gtSAMgraph.resize(0);
            // initialEstimate.clear();

            // //　更新关键帧
            // cloudPose3D.intensity = cloudKeyPoses3D->points.size(); //index
            // cloudKeyPoses3D->push_back(cloudPose3D);
       
            // cloudPose6D.intensity = cloudPose3D.intensity;
            // cloudKeyPoses6D->push_back(cloudPose6D);
            // pc_key_frames_.push_back(scan_pc_ptr_);
          
            // //更新last关键帧
            // lastKeyPose3D = cloudPose3D;
            // lastKeyPose6D = cloudPose6D;
         
            // return true;

            save_this_key_frame = true;
            
        }
    
    if (save_this_key_frame)
    {

        // update isam
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // save key frame point 注意这里要使用拷贝，否则vector中关键帧会随着scan_pc_ptr_变化
        // pcl::PointCloud<PointType>::Ptr this_key_frame(new pcl::PointCloud<PointType>());
        // pcl::copyPointCloud(*scan_pc_ptr_,  *this_key_frame);
        // pc_key_frames_.push_back(this_key_frame);
        //对关键帧降采样
        pcl::PointCloud<PointType>::Ptr this_key_frame_ds(new pcl::PointCloud<PointType>());
        downSizeFilterKeyFrames.setInputCloud(scan_pc_ptr_);
        downSizeFilterKeyFrames.filter(*this_key_frame_ds);
        pc_key_frames_.push_back(this_key_frame_ds);

        /**
         * save key poses
         */
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        
        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->points.size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().yaw();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().roll(); 
        thisPose6D.time = time_laser_odom;
        cloudKeyPoses6D->push_back(thisPose6D);
        
        // 测试优化输出的欧拉角
        // cout<< "-----latestEstimate-------" <<endl;
        // cout<< "latest xyz "<<": "<<latestEstimate.translation().x() << " " <<latestEstimate.translation().y()<<" "<<latestEstimate.translation().z()<<endl;
        // cout<< "latest rpy "<<": "<<latestEstimate.rotation().roll() << " " <<latestEstimate.rotation().pitch()<<" "<<latestEstimate.rotation().yaw()<<endl;

        // cout<< "-----origin pose-------" <<endl;
        // cout<< "point xyz "<<": "<<cloudPose6D.x << " " <<cloudPose6D.y<<" "<<cloudPose6D.z<<endl;
        // cout<< "point rpy "<<": "<<cloudPose6D.roll << " " <<cloudPose6D.pitch<<" "<<cloudPose6D.yaw<<endl;
        // cout<< "-------------------------------------------------------------------------" <<endl;
        
        // 测试输出: 注意gtsam中的欧拉角定义
        // -----latestEstimate-------
        // latest xyz : 0.0305917 -0.0420461 0.00135148
        // latest rpy : -0.0271241 4.78365e-05 0.000198124
        // -----origin pose-------
        // point xyz : 0.0305917 -0.0420461 0.00135148
        // point rpy : 0.000198124 4.78365e-05 -0.0271241

        /* 
        Scan Context loop detector 
        - ver 1: using surface feature as an input point cloud for scan context (2020.04.01: checked it works.)
        - ver 2: using downsampled original point cloud (/full_cloud_projected + downsampling)
        */
        bool usingRawCloud = true;
        if( usingRawCloud ) { // v2 uses downsampled raw point cloud, more fruitful height information than using feature points (v1)
            pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*scan_pc_ptr_,  *thisRawCloudKeyFrame);
            scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
        }

        //更新last关键帧
        lastKeyPose3D = thisPose3D;
        lastKeyPose6D = thisPose6D;



        time_insert_key_frame.toc("time insert keyFrame");
        return true;

    }else{
        return false;
    }
        
}

void BackEndOpti::correctPoses(){
    if (aLoopIsClosed == true){
        // recentCornerCloudKeyFrames. clear();
        // recentSurfCloudKeyFrames.   clear();
        // recentOutlierCloudKeyFrames.clear();
        // update key poses
        cout<< "[loop] update pose" << endl;
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i){
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
        }

        aLoopIsClosed = false;
    }
}

bool BackEndOpti::detectLoopClosure(){

    std::lock_guard<std::mutex> lock(mtx);

    /*  RS回环
    * 1. xyz distance-based radius search (contained in the original LeGO LOAM code)
    * - for fine-stichting trajectories (for not-recognized nodes within scan context search) 
    */ 
    // pcl pointcloud 中的点是按push_back的顺序储存的吗 ? 是的
    RSlatestSurfKeyFrameCloud->clear();
    RSnearHistorySurfKeyFrameCloud->clear();
    RSnearHistorySurfKeyFrameCloudDS->clear();

    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(cloudPose3D, historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

    // 与最远帧构成回环
    RSclosestHistoryFrameID = -1;
    int curMinID = 1000000;
    for (int i = 0; i < int(pointSearchIndLoop.size()); i++)
    {
        int id = pointSearchIndLoop[i];
        if (abs(cloudKeyPoses6D->points[id].time - time_laser_odom) > 30.0){
            if( id < curMinID ) {
                curMinID = id;
                RSclosestHistoryFrameID = curMinID;
            }
        }
    }

    // // 与最近帧构成回环     
    // RSclosestHistoryFrameID = -1;
    // for (int i = 0; i < int(pointSearchIndLoop.size()); i++){
    //     int id = pointSearchIndLoop[i];
    //     if (abs(cloudKeyPoses6D->points[id].time - time_laser_odom) > 30.0){
    //         RSclosestHistoryFrameID = id;
    //         break;
    //     }
    // }

    // 
    if (abs(RSclosestHistoryFrameID - (cloudKeyPoses3D->points.size() - 1)) < 10)
    {
        RSclosestHistoryFrameID = -1;
    }
    
    

    if (RSclosestHistoryFrameID == -1) //没有找到回环？
    {
        // return false;
    }else // 找到回环,保存关键帧信息
    {
        //保存当前帧最近的一个关键帧点云　一帧
        latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1; //当前关键帧id
        *RSlatestSurfKeyFrameCloud += *pcTransToInit(pc_key_frames_[latestFrameIDLoopCloure], cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
        pcl::PointCloud<PointType>::Ptr RShahaCloud(new pcl::PointCloud<PointType>());
        int cloudSize = RSlatestSurfKeyFrameCloud->points.size();
        for (int i = 0; i < cloudSize; ++i){
            if ((int)RSlatestSurfKeyFrameCloud->points[i].intensity >= 0){
                RShahaCloud->push_back(RSlatestSurfKeyFrameCloud->points[i]);
            }
        }
        RSlatestSurfKeyFrameCloud->clear();
        *RSlatestSurfKeyFrameCloud = *RShahaCloud;

        // 保存回环关键帧点云　回环关键帧左右 historyKeyframeSearchNum 的点云集合
        // 滤波
        for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j){
            if (RSclosestHistoryFrameID + j < 0 || RSclosestHistoryFrameID + j > latestFrameIDLoopCloure)
                continue;
            *RSnearHistorySurfKeyFrameCloud += *pcTransToInit(pc_key_frames_[RSclosestHistoryFrameID+j], cloudKeyPoses6D->points[RSclosestHistoryFrameID+j]);
        }
        downSizeFilterHistoryKeyFrames.setInputCloud(RSnearHistorySurfKeyFrameCloud);
        downSizeFilterHistoryKeyFrames.filter(*RSnearHistorySurfKeyFrameCloudDS);
        
        //test
        if (history_frame_pub_.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*RSnearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(time_laser_odom);
            cloudMsgTemp.header.frame_id = "/camera_init";
            history_frame_pub_.publish(cloudMsgTemp);
        }

        //>>>>>>>>test 保存关键帧与回环关键帧局部地图
        // pcl::io::savePCDFileASCII(pcd_file_path + to_string(RSclosestHistoryFrameID) + "submap.pcd", *RSnearHistorySurfKeyFrameCloudDS);
        // pcl::io::savePCDFileASCII(pcd_file_path + to_string(latestFrameIDLoopCloure) + ".pcd", *RSlatestSurfKeyFrameCloud);
        // pcl::PointCloud<PointType>::Ptr HistoryKeyFrameCloud(new pcl::PointCloud<PointType>());
        // *HistoryKeyFrameCloud += *pcTransToInit(pc_key_frames_[RSclosestHistoryFrameID], cloudKeyPoses6D->points[RSclosestHistoryFrameID]);
        // pcl::io::savePCDFileASCII(pcd_file_path + to_string(RSclosestHistoryFrameID) + ".pcd", *HistoryKeyFrameCloud);
        //<<<<<<<<<
        
        // return true;
    }


    /* 
    * 2. Scan context-based global localization 
    */
    SClatestSurfKeyFrameCloud->clear();
    SCnearHistorySurfKeyFrameCloud->clear();
    SCnearHistorySurfKeyFrameCloudDS->clear();

    // std::lock_guard<std::mutex> lock(mtx);        
    latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
    SCclosestHistoryFrameID = -1; // init with -1
    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
    SCclosestHistoryFrameID = detectResult.first;
    yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)

    // if all close, reject
    if (SCclosestHistoryFrameID == -1){ 
        return false;
    }

    // 把当前帧点云转换到　回环帧坐标系下
    *SClatestSurfKeyFrameCloud += *pcTransToInit(pc_key_frames_[latestFrameIDLoopCloure], cloudKeyPoses6D->points[SCclosestHistoryFrameID]);
    pcl::PointCloud<PointType>::Ptr SChahaCloud(new pcl::PointCloud<PointType>());
    int cloudSize = SClatestSurfKeyFrameCloud->points.size();
    for (int i = 0; i < cloudSize; ++i){
        if ((int)SClatestSurfKeyFrameCloud->points[i].intensity >= 0){
            SChahaCloud->push_back(SClatestSurfKeyFrameCloud->points[i]);
        }
    }
    SClatestSurfKeyFrameCloud->clear();
    *SClatestSurfKeyFrameCloud = *SChahaCloud;

    // 保存回环关键帧点云　回环关键帧左右 historyKeyframeSearchNum 的点云集合
    // 滤波
    for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j){
        if (SCclosestHistoryFrameID + j < 0 || SCclosestHistoryFrameID + j > latestFrameIDLoopCloure)
            continue;
        *SCnearHistorySurfKeyFrameCloud += *pcTransToInit(pc_key_frames_[SCclosestHistoryFrameID+j], cloudKeyPoses6D->points[SCclosestHistoryFrameID+j]);
    }
    downSizeFilterHistoryKeyFrames.setInputCloud(SCnearHistorySurfKeyFrameCloud);
    downSizeFilterHistoryKeyFrames.filter(*SCnearHistorySurfKeyFrameCloudDS);
    return true;
    
}

void BackEndOpti::performLoopClosure(){
    TicToc time_loop;
    if (cloudKeyPoses3D->points.empty() == true)
        return;

    if(detectLoopClosure() == false){
        return;
    }

    // make common variables at forward
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionCameraFrame;
    float noiseScore = 0.5; // constant is ok...
    gtsam::Vector Vector6(6);
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    constraintNoise = noiseModel::Diagonal::Variances(Vector6);
    robustNoiseModel = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure
        gtsam::noiseModel::Diagonal::Variances(Vector6)
    ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

    /*
     * 1. RS loop factor (radius search)
     */

    bool isValidRSloopFactor = false;

    if( RSclosestHistoryFrameID != -1 ) {
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(100);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(RSlatestSurfKeyFrameCloud);
        icp.setInputTarget(RSnearHistorySurfKeyFrameCloudDS);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        std::cout << "[RS] ICP fit score: " << icp.getFitnessScore() << std::endl;
        if ( icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore ) {
            std::cout << "[RS] Reject this loop (bad icp fit score, > " << historyKeyframeFitnessScore << ")" << std::endl;
            isValidRSloopFactor = false;
        }
        else {
            std::cout << "[RS] The detected loop factor is added between Current [ " << latestFrameIDLoopCloure << " ] and RS nearest [ " << RSclosestHistoryFrameID << " ]" << std::endl;
            isValidRSloopFactor = true;
        }

        if( isValidRSloopFactor == true ) {
            correctionCameraFrame = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)??

            //>>>>>>test
            pcl::getTranslationAndEulerAngles (correctionCameraFrame, x, y, z, roll, pitch, yaw);
            cout << "icp result: " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << endl;
            //<<<<<<test

            // pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw); // ??
            // Eigen::Affine3f correctionLidarFrame = pcl::getTransformation(z, x, y, yaw, roll, pitch); // ??
            // transform from world origin to wrong pose
            Eigen::Affine3f t_wrong_to_init = pclPointToAffine3f(cloudKeyPoses6D->points[latestFrameIDLoopCloure]);

            //>>>>>>test
            pcl::getTranslationAndEulerAngles (t_wrong_to_init, x, y, z, roll, pitch, yaw);
            cout << "t_currentOld_to_init: " <<latestFrameIDLoopCloure<< " id " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << endl;
            //<<<<<<<

            // transform from world origin to corrected pose
            Eigen::Affine3f tCorrect = correctionCameraFrame * t_wrong_to_init; // pre-multiplying -> successive rotation about a fixed frame
            pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);

            //>>>>>>test
            cout << "t_currentNew_to_init: " <<latestFrameIDLoopCloure<< " id " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << endl;
            //<<<<<<<

            // gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z)); // ??
            gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(yaw, pitch, roll), Point3(x, y, z));
            gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(cloudKeyPoses6D->points[RSclosestHistoryFrameID].yaw, 
                                                    cloudKeyPoses6D->points[RSclosestHistoryFrameID].pitch, 
                                                    cloudKeyPoses6D->points[RSclosestHistoryFrameID].roll),
                                            Point3(cloudKeyPoses6D->points[RSclosestHistoryFrameID].x, 
                                                    cloudKeyPoses6D->points[RSclosestHistoryFrameID].y, 
                                                    cloudKeyPoses6D->points[RSclosestHistoryFrameID].z));

            //>>>>>>test
            cout << "t_loop_to_init: " << RSclosestHistoryFrameID << " id "
                                       << cloudKeyPoses6D->points[RSclosestHistoryFrameID].x << " " 
                                       << cloudKeyPoses6D->points[RSclosestHistoryFrameID].y << " " 
                                       << cloudKeyPoses6D->points[RSclosestHistoryFrameID].z << " " 
                                       << cloudKeyPoses6D->points[RSclosestHistoryFrameID].roll << " " 
                                       << cloudKeyPoses6D->points[RSclosestHistoryFrameID].pitch << " " 
                                       << cloudKeyPoses6D->points[RSclosestHistoryFrameID].yaw << " " << endl;
            //<<<<<<<<

            gtsam::Vector Vector6(6);
            // float noiseScore = icp.getFitnessScore();
            // Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
            // constraintNoise = noiseModel::Diagonal::Variances(Vector6);

            std::lock_guard<std::mutex> lock(mtx);
            gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, RSclosestHistoryFrameID, poseFrom.between(poseTo), robustNoiseModel));
            // lego 源码
            // gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, RSclosestHistoryFrameID, poseFrom.between(poseTo), constraintNoise));
            isam->update(gtSAMgraph);
            isam->update();
            gtSAMgraph.resize(0);
            aLoopIsClosed = true;
            // 可视化回环关系
            if (loop_line_pub_.getNumSubscribers() != 0)
            {
                geometry_msgs::Point pose_current;
                geometry_msgs::Point pose_loop;
                pose_current.x = cloudKeyPoses3D->points[latestFrameIDLoopCloure].x;
                pose_current.y = cloudKeyPoses3D->points[latestFrameIDLoopCloure].y;
                pose_current.z = cloudKeyPoses3D->points[latestFrameIDLoopCloure].z;

                pose_loop.x = cloudKeyPoses3D->points[RSclosestHistoryFrameID].x;
                pose_loop.y = cloudKeyPoses3D->points[RSclosestHistoryFrameID].y;
                pose_loop.z = cloudKeyPoses3D->points[RSclosestHistoryFrameID].z;
                
                line.points.push_back(pose_current);
                line.points.push_back(pose_loop);

                loop_line_pub_.publish(line);
            }
            
        }
    }// RS loop factor end;

    /*
    * 2. SC loop factor (scan context)
    */
    bool isValidSCloopFactor = false;
    if( SCclosestHistoryFrameID != -1 ) {
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(100);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        icp.setInputSource(SClatestSurfKeyFrameCloud);
        icp.setInputTarget(SCnearHistorySurfKeyFrameCloudDS);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result); 

        std::cout << "[SC] ICP fit score: " << icp.getFitnessScore() << std::endl;
        if ( icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore ) {
            std::cout << "[SC] Reject this loop (bad icp fit score, > " << historyKeyframeFitnessScore << ")" << std::endl;
            isValidSCloopFactor = false;
        }
        else {
            std::cout << "[SC] The detected loop factor is added between Current [ " << latestFrameIDLoopCloure << " ] and SC nearest [ " << SCclosestHistoryFrameID << " ]" << std::endl;
            isValidSCloopFactor = true;
        }

        if( isValidSCloopFactor == true ) {
            correctionCameraFrame = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
            pcl::getTranslationAndEulerAngles (correctionCameraFrame, x, y, z, roll, pitch, yaw);
            gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(yaw, pitch, roll), Point3(x, y, z));
            gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
            
            std::lock_guard<std::mutex> lock(mtx);
            // gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, closestHistoryFrameID, poseFrom.between(poseTo), constraintNoise)); // original 
            gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, SCclosestHistoryFrameID, poseFrom.between(poseTo), robustNoiseModel)); // giseop
            isam->update(gtSAMgraph);
            isam->update();
            gtSAMgraph.resize(0);

            aLoopIsClosed = true;

            // 可视化回环关系
            if (loop_line_pub_.getNumSubscribers() != 0)
            {
                geometry_msgs::Point pose_current;
                geometry_msgs::Point pose_loop;
                pose_current.x = cloudKeyPoses3D->points[latestFrameIDLoopCloure].x;
                pose_current.y = cloudKeyPoses3D->points[latestFrameIDLoopCloure].y;
                pose_current.z = cloudKeyPoses3D->points[latestFrameIDLoopCloure].z;

                pose_loop.x = cloudKeyPoses3D->points[SCclosestHistoryFrameID].x;
                pose_loop.y = cloudKeyPoses3D->points[SCclosestHistoryFrameID].y;
                pose_loop.z = cloudKeyPoses3D->points[SCclosestHistoryFrameID].z;
                
                line.points.push_back(pose_current);
                line.points.push_back(pose_loop);

                loop_line_pub_.publish(line);
            }
        }

    }// CS loop factor end; 



    time_loop.toc("time loop");
}

void BackEndOpti::loopClosureThread(){

    // if (loopClosureEnableFlag == false)
    //     return;

    ros::Rate rate(1);
    while (ros::ok()){
        rate.sleep();
        performLoopClosure();
    }
} // loopClosureThread

void BackEndOpti::visualizeGlobalMapThread(){
    ros::Rate rate(0.2);
    while (ros::ok()){
        rate.sleep();
        publishGlobalMap();
    }
    // save final point cloud

    // TransformAndSaveMap();

}

void BackEndOpti::publishGlobalMap(){

    if (global_map_pub_.getNumSubscribers() == 0)
        return;

    if (cloudKeyPoses3D->points.empty() == true)
        return;

    // kd-tree to find near key frames to visualize
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    // search near key frames to visualize
    mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(cloudPose3D, globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    for (int i = 0; i < int(pointSearchIndGlobalMap.size()); i++)
    {
        globalMapKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    }

    for (int i = 0; i < int(globalMapKeyPoses->points.size()); i++)
    {
        int thisKeyId = globalMapKeyPoses->points[i].intensity;
        *globalMapKeyFrames += *pcTransToInit(pc_key_frames_[thisKeyId], cloudKeyPoses6D->points[thisKeyId]);
    }

    // downsample visualized points
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    
    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*globalMapKeyFramesDS, cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().fromSec(time_laser_odom);
    cloudMsgTemp.header.frame_id = "/camera_init";
    global_map_pub_.publish(cloudMsgTemp);

    globalMapKeyPoses->clear();
    globalMapKeyPosesDS->clear();
    globalMapKeyFrames->clear();
    globalMapKeyFramesDS->clear();    

}

void BackEndOpti::run(){


    // if (timeSyncData())
    // {
    //     cout << "time sync :  ture" <<endl;
    //     if (insertKeyFrame())
    //     {
    //         cout << "insertKeyFrame : true"  <<endl;
    //     }
    //     correctPoses();
    //     publishKeyPosesAndFrames();
    // }

    cout << "=======" <<endl;
    
    if (timeSyncData())
    {
        insertKeyFrame();
        correctPoses();
        publishKeyPosesAndFrames();
        cout << "-----------------------------------" << endl;
    }
  
}

// class BackEndOpti end;
//-----------------------------------------------------------------------------------------------//



int main(int argc, char **argv)
{
    ros::init(argc, argv, "BackEndOpti");
    ROS_INFO("\033[1;32m---->\033[0m BackEndOpti Node Started.");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate rate(30.0);

    BackEndOpti backendopti(nh, nh_);

    std::thread loopthread(&BackEndOpti::loopClosureThread, &backendopti);
    std::thread visualizeMapThread(&BackEndOpti::visualizeGlobalMapThread, &backendopti);

    while(ros::ok())
    {
        backendopti.run();
        ros::spinOnce();
        rate.sleep();
    }

    loopthread.join();
    visualizeMapThread.join();

    return 0;

}


