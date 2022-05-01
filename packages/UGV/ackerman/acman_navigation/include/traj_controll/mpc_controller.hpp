

#include <cmath>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <traj_controll/data_utils.h>

class MPC_Controller{
public:
    MPC_Controller(ros::NodeHandle& nh, nav_msgs::Path::ConstPtr path_ptr, State* state_ptr);
    ~MPC_Controller();
    CMD solve();           // 求解控制量的
    void reset();          // 重置路径和检索
private:
    // 输入路径和车辆状态，通过指针拉出去。
    ros::NodeHandle nh_;
    nav_msgs::Path::ConstPtr path_ptr_;
    State* state_ptr_;
    State ref_state_;
    double ref_vel_;       // 根据
    double ref_deta;
 
    int index_;            // 参考路径点的 index
    // MPC 预设参数
    const int Nx  = 3;     // 状态量的个数
    const int Nu  = 2;     // 控制量个数
    const int Np  = 60;    // 预测步长
    const int Nc  = 30;    // 控制步长
    const int row = 10;    // TODO: 松弛因子

    // 控制量约束条件
    Eigen::Vector2d umin, umax, delta_umin, delta_umax;

    // 运动学误差状态空间方程的相关矩阵
    Eigen::Matrix<double, 3, 3> A;    // 状态转移矩阵
    Eigen::Matrix<double, 3, 2> B;    // 控制矩阵

    // 构造的新的状态空间矩阵
    Eigen::MatrixXd kesi;        // 增广的状态量
    Eigen::MatrixXd A_Enlarge;   // 增广的矩阵A
    Eigen::MatrixXd B_Enlarge;   // 增广的矩阵B
    Eigen::MatrixXd C_Enlarge;   // 输出矩阵，[I,0] 屏蔽控制量 [3x5]*[5x1]

    // Y = PHI*kesi + THETA*delta_U  其中 Y 是预测步长的所有状态 因此这个等式十分庞大            
    Eigen::MatrixXd PHI;         // 可以理解为预测步长的状态转移矩阵
    Eigen::MatrixXd THETA;       // 可以理解为预测步长过程中的控制矩阵，控制量此时是真实控制量的二阶微分

    // 二次型目标函数的相关矩阵
    // H 矩阵中和了Q\R和THETA, +1 是多了一行松弛因子
    Eigen::MatrixXd Q, R, H;     // Q 矩阵约束状态收敛; R 矩阵约束控制量    
    
    Eigen::MatrixXd E, G;        // E = PHI*Kesi  G = E*Q*THETA， 为了保持和 H 矩阵纬度相同

    // 约束条件的矩阵  A_l 是一个特殊形式的 下三角方阵
    Eigen::MatrixXd A_l;         // U = Ut + A_l*delt_U ,所以 A_l就和控制量本身的最大最小值共同约束了
    Eigen::MatrixXd Ut;          // Ut 是将 U(K-1)两个控制量全部变成一列 
    Eigen::MatrixXd delta_U;     // 新的控制量每一步的增量，也就是二次规划求解的最后结果。

    // 控制量和控制量变化量的约束 四个克罗内克积张量, 常量矩阵
    Eigen::MatrixXd Umin, Umax, delt_Umin, delt_Umax;

    // TODO: 构造二次规划求解器

private:    
    int calc_target_index();          // 计算路径参考点
    double refState(int index);       // 参考点的速度 v, 曲率半径， 航向Yaw角
    double latError();                // 计算横向误差
    void update_matrix();             // 更新状态矩阵

};


MPC_Controller::MPC_Controller(ros::NodeHandle& nh, nav_msgs::Path::ConstPtr path_ptr, State* state_ptr)
:nh_(nh)
{
    path_ptr_  = path_ptr;
    state_ptr_ = state_ptr;

    // 常量矩阵初始化
    // 控制量约束条件
    umin << -0.2, -0.3;
    umax << 0.2, 0.3;
    delta_umin << -0.05, -0.5;
    delta_umax << 0.05, 0.5;

    Q = Eigen::MatrixXd::Identity(Nx*Np,Nx*Np); 
    R = Eigen::MatrixXd::Identity(Nc*Nu,Nc*Nu);

}

MPC_Controller::~MPC_Controller(){
    if(path_ptr_ != nullptr){
        path_ptr_ = nullptr;
    }
    if(state_ptr_ != nullptr){
        state_ptr_ = nullptr;
    }
    std::cout<<"MPC_Controller destruct "<<std::endl;
}

int MPC_Controller::calc_target_index(){
    double dis_pow = pow(path_ptr_->poses[0].pose.position.x - state_ptr_->x,2)+
                 pow(path_ptr_->poses[0].pose.position.y - state_ptr_->y, 2);
    int index = 0;
    int i = 0;
    double temp;
    for(auto _pose : path_ptr_->poses){
        temp = pow(_pose.pose.position.x - state_ptr_->x, 2)+
               pow(_pose.pose.position.y - state_ptr_->y, 2);
        if(temp < dis_pow){
            dis_pow = temp;
            index = i;
        }
        i++;
    }
    return index;
}

double MPC_Controller::refState(int index){


}

CMD MPC_Controller::solve(){
    update_matrix();
    


}
