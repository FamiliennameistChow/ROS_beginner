/********************************
 * TrajOptimizer.hpp
 *
 * Author: Born Chow
 * 
 * Date: 2020.09.10 
 *
 * Description:
 * 
 * 使用多项式描述轨迹
 * 
 * 使用QPOASES求解二次规划QP问题
 *
 * Refer:
 * https://blog.csdn.net/qq_41324346/article/details/106768885?utm_medium=distribute.pc_relevant.none-task-blog-title-2&spm=1001.2101.3001.4242
 * https://blog.csdn.net/Q_upup/article/details/106442109?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.channel_param
 * https://blog.csdn.net/weixin_40709533/article/details/86064148
 * 
 * 
 * 
 **********************************/

# include <iostream>
# include <vector>
# include <eigen3/Eigen/Dense>
# include <qpOASES.hpp>
# include <math.h>
# include "data_type.h"
# include "tictoc.h"

using namespace std;
USING_NAMESPACE_QPOASES
// #define DEBUG ;

class QPOASESSlover
{
// **************************************************
//                 min P^T * Q * P
//                b_lo <= A * P <= b_up
//                   
// *************************************************
private:
    int n; // 多项式的阶数 order
    int k; //轨迹的段数

    int Num_sigle_Ctrl; //每段轨迹的控制点数　这里一个坐标(x,y,z)作为一个控制点？？
    int Num_total_ctrl; //全部轨迹的控制点数

    int Num_equality_constraint; //等式约束个数
    int Num_inequality_constraint; //不等式约束个数

    int minimum_order; //表征是最小化acc, jerk, snap 的参数; 2 = acc; 3 = jerk; 4 = snap;

    Eigen::MatrixXd Q;
    Eigen::MatrixXd P;
    Eigen::MatrixXd M_equality_constraint; //等式约束矩阵
    Eigen::MatrixXd b_equality_constraint; //等式约束边界

    Eigen::MatrixXd M_inequality_constraint; //不等式约束矩阵
    Eigen::MatrixXd b_inequality_constraint; //不等式约束边界

    Eigen::MatrixXd A;

    Eigen::MatrixXd pos = Eigen::MatrixXd::Zero(2,3);
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2,3);
    Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2,3);

    Eigen::MatrixXd pva = Eigen::MatrixXd::Zero(2,3);

    vector<double> T; //时间序列
    vector<Cube> corridor_;

    bool qp_solved;

public:
    QPOASESSlover(int order, int min_order, vector<double> time_list, vector<Eigen::Vector3d> path, Eigen::MatrixXd Vel, Eigen::MatrixXd Acc);
    QPOASESSlover();
    int factorialNum(int a); // 返回a的阶乘 a!
    bool solve();
    void initParam(int order, int min_order, vector<Cube> corridor, vector<Eigen::Vector3d> path, Eigen::MatrixXd Vel, Eigen::MatrixXd Acc);
    Eigen::MatrixXd getResultP();
    vector<double> getTimeList();
    Eigen::MatrixXd createRowVector(double time, bool positive, int pva); //产生某一时刻的时间序列 [1, t, t^2, t^3,.....t^n]
    ~QPOASESSlover();
};


QPOASESSlover::QPOASESSlover() 
{
    // //>>>>>>>>>>>>>test<<<<<<<<<<<<<<<<<
    // n = 7;
    // k = 4;
    // T = {0, 1, 3, 4, 5};
    // cout << ".....in test....." << endl;
    // cout << "4! " << factorialNum(4) << endl;
    // cout << "T : { ";
    // for (auto it = T.begin(); it != T.end(); it++)
    // {
    //     cout << *it << " ";
    // }
    // cout <<"}"<< endl;

    // cout << T.front() << " " << T.back() << endl;

    // // 起始位置的 p ,v ,a 信息
    // pos.row(0) << 0, 0, 0;
    // pos.row(1) << 10, 5, 0;
    // vel.row(0) << 0, 0, 0;
    // acc.row(0) << 0, 0, 0;
    // vel.row(1) << 0, 0, 0;
    // acc.row(1) << 0, 0, 0;


    // //>>>>>>>>>>>>>>>test<<<<<<<<<<<<<<<

    // // 初始化变量
    // Num_sigle_Ctrl = n + 1; //每段轨迹的控制点数　这里一个坐标(x,y,z)作为一个控制点？？
    // Num_total_ctrl = Num_sigle_Ctrl * k; //全部轨迹的控制点数

    // Num_equality_constraint = (k + 1) * 3; //等式约束个数
    // Num_inequality_constraint = 0; //不等式约束个数

    // minimum_order = 3; //表征是最小化acc, jerk, snap 的参数; 2 = acc; 3 = jerk; 4 = snap;

    // //初始化矩阵
    // Q = Eigen::MatrixXd::Zero(Num_total_ctrl, Num_total_ctrl);
    // P = Eigen::MatrixXd::Zero(Num_total_ctrl, 3);
    // M_equality_constraint = Eigen::MatrixXd::Zero(Num_equality_constraint, Num_total_ctrl); //等式约束矩阵
    // b_equality_constraint = Eigen::MatrixXd::Zero(Num_equality_constraint, 3); //等式约束边界

    // M_inequality_constraint = Eigen::MatrixXd::Zero(Num_inequality_constraint, Num_total_ctrl); //不等式约束矩阵
    // b_inequality_constraint = Eigen::MatrixXd::Zero(Num_inequality_constraint, 3); //不等式约束边界

    // // A = Eigen::MatrixXd::Zero(Num_equality_constraint, Num_total_ctrl);

}

QPOASESSlover::~QPOASESSlover()
{

}

void QPOASESSlover::initParam(int order, int min_order, vector<Cube> corridor, vector<Eigen::Vector3d> path, Eigen::MatrixXd Vel, Eigen::MatrixXd Acc){

    cout << "=================QP solver param================" << endl;
    n = order;  // 多项式的阶数 order
    k = corridor.size();  //轨迹的段数
    T.clear();
    T.push_back(0);
    double sum = 0;

    for (int i = 0; i < corridor.size(); i++)
    {
        // sum += round(corridor[i].t*100)/100; // 保留两位小数
        sum += corridor[i].t;
        T.push_back(sum);
    }

    corridor_ = corridor;
    
    // 起始位置的 p ,v ,a 信息
    pos.row(0) = path.front();
    pos.row(1) = path.back();
    vel.row(0) = Vel.row(0);
    acc.row(0) = Acc.row(0);
    vel.row(1) = Vel.row(1);
    acc.row(1) = Acc.row(1);



    // 初始化变量
    Num_sigle_Ctrl = n + 1; //每段轨迹的控制点数　这里一个坐标(x,y,z)作为一个控制点？？
    Num_total_ctrl = Num_sigle_Ctrl * k; //全部轨迹的控制点数

    Num_equality_constraint = (k + 1) * 3; //等式约束个数
    Num_inequality_constraint = k - 1; //不等式约束个数

    minimum_order = min_order; //表征是最小化acc, jerk, snap 的参数; 2 = acc; 3 = jerk; 4 = snap;
    qp_solved = false;

    //初始化矩阵
    Q = Eigen::MatrixXd::Zero(Num_total_ctrl, Num_total_ctrl);
    P = Eigen::MatrixXd::Zero(Num_total_ctrl, 3);
    M_equality_constraint = Eigen::MatrixXd::Zero(Num_equality_constraint, Num_total_ctrl); //等式约束矩阵
    b_equality_constraint = Eigen::MatrixXd::Zero(Num_equality_constraint, 3); //等式约束边界

    M_inequality_constraint = Eigen::MatrixXd::Zero(Num_inequality_constraint, Num_total_ctrl); //不等式约束矩阵
    b_inequality_constraint = Eigen::MatrixXd::Zero(Num_inequality_constraint, 6); //不等式约束边界

    // A = Eigen::MatrixXd::Zero(Num_equality_constraint, Num_total_ctrl);

    cout <<">>>> T                     : { ";
    for (auto it = T.begin(); it != T.end(); it++)
    {
        cout << *it << " ";
    }
    cout <<"} ";

    cout << T.front() << " --> " << T.back() << endl;
  
    cout <<">>>> polynomial order      : " << n << endl;
    cout <<">>>> segment Num           : " << k << endl;
    cout <<">>>> minimum order         : " << minimum_order << " [3 = jerk; 4 = snap] "<<endl;
    cout <<">>>> pos of first and last : \n" << pos << endl;
    cout <<">>>> vel of first and last : \n" << vel << endl;
    cout <<">>>> acc of first and last : \n" << Acc << endl;
    cout << "=================QP solver param================" << endl;

}

// 返回a的阶乘 a!
int QPOASESSlover::factorialNum(int a){
    int sum = 1;
    int i = 2;
    if (a == 0 || a == 1)
    {
        return 1;
    }

    while (i <= a)
    {
        sum *= i;
        i++;
    }

    return sum;
}

bool QPOASESSlover::solve(){
    //>>>>>>>>>test<<<<<<<<

    // cout << "test createRowVector: "<<endl;
    // Eigen::MatrixXd s;
    // cout << "-----0000-------" << endl;
    // s = createRowVector(2, true, 0);
    // cout <<"s:\n" << s <<endl;

    //>>>>>>>>>>>>>>test<<<<<<<<<<<<

    //**********************************************************
    // create Q matrix
    // 即是求优化目标函数的海塞矩阵; 最终的Q matrix是分段Qi矩阵的组合 k为段数
    //            
    //                | Q1           |
    //                |   Q2         |
    //            Q = |     ...      |
    //                |        ...   |
    //                |           Qk |
                
    //*********************************************************
    cout << "Q matrix set......" << endl;
    Eigen::MatrixXd Qi = Eigen::MatrixXd::Zero(Num_sigle_Ctrl, Num_sigle_Ctrl);
    for (size_t i = 1; i <= k; i++)
    {
        for (int r = minimum_order; r < Qi.rows(); r++)
        {
            for (int c = minimum_order; c < Qi.cols(); c++)
            {
                long double value 
                         = (factorialNum(r) / factorialNum(r-minimum_order)) * 
                           (factorialNum(c) / factorialNum(c-minimum_order)) * 
                           (1.0 / (c + r - 2*minimum_order + 1)) * (pow(T[i], c+r-2*minimum_order+1) - pow(T[i-1], c+r-2*minimum_order+1));
                Qi(r, c) = value;
                
                #ifdef DEBUG
                // debug output显示
                cout << "Qi(r, c) -->                                                           : " << r << " , " << c <<" "<< Qi(r,c) <<" --> " << value<< endl;
                cout << "factorialNum(r) / factorialNum(r-minimum_order)                        : " << factorialNum(r) / factorialNum(r-minimum_order) << endl;
                cout << "factorialNum(c) / factorialNum(c-minimum_order)                        : " << factorialNum(c) / factorialNum(c-minimum_order) << endl;
                cout << "1.0 / (c + r - 2*minimum_order + 1)                                    : " << 1.0 / (c + r - 2*minimum_order + 1) << endl;
                cout << " pow(T[i], c+r-2*minimum_order+1) - pow(T[i-1], c+r-2*minimum_order+1) : " << pow(T[i], c+r-2*minimum_order+1) - pow(T[i-1], c+r-2*minimum_order+1) << endl;
                cout << "------------------------------------------------------" << endl;
                #endif
            }
            
        }
        cout << "Qi :\n" << Qi << endl;

        #ifdef DEBUG
        // debug output　显示
        Eigen::EigenSolver<Eigen::MatrixXd> es(Qi);
        Eigen::MatrixXd D = es.pseudoEigenvalueMatrix();
        cout <<" Qi D  : \n" << D << endl;
        #endif

        Q.block((i-1)*Num_sigle_Ctrl, (i-1)*Num_sigle_Ctrl, Num_sigle_Ctrl, Num_sigle_Ctrl) = Qi;
    }

    cout << "Q :\n" << Q << endl;
    cout << "Q size: " << Q.rows() <<" x " << Q.cols() << endl;

    #ifdef DEBUG
    ////判断Q 特征值 若所有特征值均不小于零，则称为半正定。
    Eigen::EigenSolver<Eigen::MatrixXd> es(Q);
    Eigen::MatrixXd D = es.pseudoEigenvalueMatrix();
    cout <<" D  : \n" << D << endl;
    #endif

    // ********************************************************
    // create M_equality_constraint, 共计(k+1)*3个等式约束
    // 等式约束, 
    // 先存放初始位置和结束位置上的 p , v , a 等式关系; 共计 2 * 3 = 6个
    // 然后存放中间位置的 p , v , a 关系相等约束关系 共计 (k-1) * 3 个
    // ********************************************************

    //Eigen::MatrixXd single_one_vector = Eigen::MatrixXd::Ones(1, Num_sigle_Ctrl); //test

    for (int i = 0; i < M_equality_constraint.rows(); i++)
    {
        if (i < 3) //起点约束
        {
            M_equality_constraint.row(i).block(0, 0, 1, Num_sigle_Ctrl) = createRowVector(T.front(), true, i);
        }

        if (i >= 3 && i < 6) //终点约束
        {
            M_equality_constraint.row(i).block(0, (k-1)*Num_sigle_Ctrl, 1, Num_sigle_Ctrl) = createRowVector(T.back(), true, i-3); //
        }
        
        if (i >= 6) //中间点约束
        {
            M_equality_constraint.row(i).block(0, Num_sigle_Ctrl*((i-6)/3), 1, Num_sigle_Ctrl) = createRowVector(T[(i-6)/3+1], true,  i%3);
            M_equality_constraint.row(i).block(0, Num_sigle_Ctrl*((i-6)/3+1), 1, Num_sigle_Ctrl) = createRowVector(T[(i-6)/3+1], false, i%3);
        }
        
    }

    cout << "M_equality_constraint SIZE " << M_equality_constraint.rows() <<" x " << M_equality_constraint.cols()<< endl;
    cout << "M_equality_constraint: \n" << M_equality_constraint << endl;


    //********************************************
    // create b_equality_constraint 等式约束边界矩阵
    //  M * P = b   b为(k+1)*3 x 3维 向量
    // 前6维为起始和终止位置的 p, v, a
    //********************************************
    
    // for (int i = 0; i < b_equality_constraint.rows(); i++)
    // {
    //     if (i < 3) //起点约束边界
    //     {
    //         b_equality_constraint.row(i) = pos.row(i);
    //     }
        
    //     if (i >= 3 && i < 6) //终点约束边界
    //     {
    //         b_equality_constraint(i, 0) = pva(1, i%3);
    //     }
    // }

    b_equality_constraint.row(0) = pos.row(0);
    b_equality_constraint.row(1) = vel.row(0);
    b_equality_constraint.row(2) = acc.row(0);
    b_equality_constraint.row(3) = pos.row(1);
    b_equality_constraint.row(4) = vel.row(1);
    b_equality_constraint.row(5) = acc.row(1);
   
    cout << "b_equality_constraint: \n" << b_equality_constraint << endl;

    //************************************************
    // 创建　M_inequality_constraint　不等式约束, 目前约束中间点的位置在corridor重叠区域内， 共计 (k-1) 个
    // b_i_lo　< M_i < b_i_up
    //
    //************************************************
    for (int i = 1; i <= M_inequality_constraint.rows(); i++)
    {
        M_inequality_constraint.row(i-1).block(0, i*Num_sigle_Ctrl, 1, Num_sigle_Ctrl) = createRowVector(T[i], true, 0);
    }
    
    cout << "M_inequality_constraint SIZE " << M_inequality_constraint.rows() <<" x " << M_inequality_constraint.cols()<< endl;
    cout << "M_inequality_constraint: \n" << M_inequality_constraint << endl;

    //************************************************
    // 创建 b_inequality_constraint 不等式约束边界矩阵 (k-1) * 6
    // (x_lo , x_up, y_lo, y_up, z_lo, z_up)
    //
    //************************************************
    for (int i = 1; i < corridor_.size(); i++)
    {
        double x_lo , x_up, y_lo, y_up, z_lo, z_up;
        x_lo = max(corridor_[i-0].vertex(3, 0), corridor_[i].vertex(3, 0));
        x_up = min(corridor_[i-1].vertex(0, 0), corridor_[i].vertex(0, 0));

        if (x_lo > x_up)
        {
            double temp = x_lo;
            x_lo = x_up;
            x_up = temp;
        }
        

        y_lo = max(corridor_[i-0].vertex(0, 1), corridor_[i].vertex(0, 1));
        y_up = min(corridor_[i-1].vertex(1, 1), corridor_[i].vertex(1, 1));

        if (y_lo > y_up)
        {
            double temp = y_lo;
            y_lo = y_up;
            y_up = temp;
        }

        z_lo = max(corridor_[i-0].vertex(4, 2), corridor_[i].vertex(4, 2));
        z_up = min(corridor_[i-1].vertex(0, 2), corridor_[i].vertex(0, 2));

        if (z_lo > z_up)
        {
            double temp = z_lo;
            z_lo = z_up;
            z_up = temp;
        }

        b_inequality_constraint.row(i-1) << x_lo, x_up, y_lo, y_up, z_lo, z_up;
    }

    cout << "b_inequality_constraint: \n" << b_inequality_constraint << endl;
    
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>data prepared end<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //************************************************
    // 创建QProblem类的实例 
    // QProblem( int_t nV, int_t nC );  
    // 变量数nV   约束数nC
    //************************************************
    QProblem qp_solver(Num_total_ctrl, (Num_equality_constraint+Num_inequality_constraint), qpOASES::HST_SEMIDEF);


    // double* h_matrix = Q.data();

    int kNumOfMatrixElements = Q.cols()*Q.rows();
    double h_matrix[kNumOfMatrixElements];
    int index = 0;
    for (int r = 0; r < Q.rows(); r++)
    {
        for (int c = 0; c < Q.cols(); c++)
        {
            h_matrix[index] = Q(r,c);
            index++;
        } 
    }

    #ifdef DEBUG
    /////显示
    cout << "h_matrix: ";
    for (int i = 0; i < kNumOfMatrixElements; i++)
    {
        cout << h_matrix[i] << " ";
    }
    cout << endl;
    #endif
    
    // 等式约束
    double Affine_constraint_matrix[Num_total_ctrl * (Num_equality_constraint+Num_inequality_constraint)];  // NOLINT 大小为参数个数乘以约束个数
    double constraint_lower_bound[Num_equality_constraint+Num_inequality_constraint];                // NOLINT 大小为约束条件个数
    double constraint_upper_bound[Num_equality_constraint+Num_inequality_constraint];                // NOLINT 大小为约束条件个数
    
    //double* Affine_constraint_matrix = M_equality_constraint.data();

    index = 0;
    for (int r = 0; r < M_equality_constraint.rows(); r++)
    {
        for (int c = 0; c < M_equality_constraint.cols(); c++)
        {
            Affine_constraint_matrix[index] = M_equality_constraint(r,c);
            index++;
        } 
    }


    for (int r = 0; r < M_inequality_constraint.rows(); r++)
    {
        for (int c = 0; c < M_inequality_constraint.cols(); c++)
        {
            Affine_constraint_matrix[index] = M_inequality_constraint(r,c);
            index++;
        }
        
    }
    

    #ifdef DEBUG
    cout << "Affine_constraint_matrix: \n";
    for (int i = 0; i < Num_total_ctrl * Num_equality_constraint; i++)
    {
        cout << Affine_constraint_matrix[i] << " ";
    }
    cout << endl;
    #endif

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(Num_total_ctrl, 1);
    
    double* g_matrix = G.data();
    double* lb = nullptr;
    double* ub = nullptr;

    // 求三个维度的P向量
    int solved_xyz = 0;
    for (int xyz = 0; xyz < 2; xyz++)
    {
        cout << "slove :" << xyz << " [0 - x , 1 - y, 2 - z]" << endl;
        index = 0;
        for (int r = 0; r < M_equality_constraint.rows(); ++r) {
            constraint_lower_bound[r] = b_equality_constraint(r, xyz); // 上下限相同
            constraint_upper_bound[r] = b_equality_constraint(r, xyz);
            index++;
        }

        for (int r = 0; r < M_inequality_constraint.rows(); ++r)
        {
            constraint_lower_bound[r+index] = b_inequality_constraint(r, xyz*2);
            constraint_upper_bound[r+index] = b_inequality_constraint(r, xyz*2+1);
        }
        

        int_t nWSR = 1000;

        returnValue ret;

        if (xyz == 0) //初始化
        {
            ret = qp_solver.init(h_matrix, g_matrix, Affine_constraint_matrix, lb, ub, constraint_lower_bound, constraint_upper_bound, nWSR);
        }else // 热启动
        {
            ret = qp_solver.hotstart(g_matrix, lb, ub, constraint_lower_bound, constraint_upper_bound, nWSR);
        }

        #ifdef DEBUG
        cout << "number of varibles:             " << qp_solver.getNV() << endl;
        cout << "number of constraints:          " << qp_solver.getNC() << endl;
        cout << "number of equality constraints: " << qp_solver.getNEC() << endl;
        #endif
        
        if (ret != qpOASES::SUCCESSFUL_RETURN)
        {
            cout << "[qpOASES]: slover failed" << endl;
            qp_solved = false;
            
        }else
        {
            double result[Num_total_ctrl];  // NOLINT
            memset(result, 0, sizeof result);      //全为0
            qp_solver.getPrimalSolution(result);  //获取结果

            // cout << "result " << endl;
            for (size_t i = 0; i < Num_total_ctrl; i++)
            {
                P(i, xyz) = result[i];
                // cout << result[i] << " ";
            }
            // cout << endl;
            qp_solved = true;
            solved_xyz++;

        }   
    }

    // cout << "P: \n" << P << endl;
    cout << "solved_xyz " << solved_xyz << endl;

    if (solved_xyz == 2)
    {
       return true;
    }else
    {
        return false;
    }
    
    
 
}



Eigen::MatrixXd QPOASESSlover::createRowVector(double time, bool positive, int pva){
    Eigen::MatrixXd single_time_vector = Eigen::MatrixXd::Zero(1, Num_sigle_Ctrl);
    // single_time_vector = Eigen::MatrixXd::Zero(1, Num_sigle_Ctrl);
    if (pva == 0)
    {
        for (int i = 0; i < Num_sigle_Ctrl; i++)
        {
            single_time_vector(0, i) = pow(time, i);
        }

    }else if (pva == 1)
    {
        for (int i = 0; i < Num_sigle_Ctrl; i++)
        {
            if (i - pva < 0)
            {
                single_time_vector(0, i) = 0;
            }else
            {
                single_time_vector(0, i) = i*pow(time, i-pva);
            }
        }

    }else if (pva == 2)
    {
        for (int i = 0; i < Num_sigle_Ctrl; i++)
        {
            if (i - pva < 0)
            {
                single_time_vector(0, i) = 0;
            }else
            {
                single_time_vector(0, i) = i*(i-1)*pow(time, i-pva);
            }
        }

    }else
    {
        cout << "!!!ERROR: ONLY support use p, v, a constraint NOW"<< endl;
        return single_time_vector;
    }

    if (positive)
    {
        return single_time_vector;
    }else
    {
        return single_time_vector * (-1);
    }
}

Eigen::MatrixXd QPOASESSlover::getResultP(){
    if (qp_solved)
    {
        return P;
    }else
    {
        cout << "!!!ERROR: slover failed NO P matrix" << endl;
    } 
}


vector<double> QPOASESSlover::getTimeList(){
    return T;
}


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>class end<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// int main()
// {
//     // use test 
//     vector<double> time_list = {0, 1, 3, 4, 5, 6};
//     int poly_order = 7;
//     int segment_Num = 5;
//     vector<Eigen::Vector3d> path_test;
//     Eigen::Vector3d pos;

//     Eigen::MatrixXd vel_test = Eigen::MatrixXd::Zero(2,3);
//     Eigen::MatrixXd acc_test = Eigen::MatrixXd::Zero(2,3);

//     for (size_t i = 0; i < 5; i++)
//     {   
//         pos << i, i, i;
//         path_test.push_back(pos);
//     }
    

//     vel_test.row(0) << 0, 0, 0;
//     acc_test.row(0) << 0, 0, 0;
//     vel_test.row(1) << 0, 0, 0;
//     acc_test.row(1) << 0, 0, 0;

//     QPOASESSlover qpslover;
//     qpslover.initParam(poly_order, 3, time_list, path_test, vel_test, acc_test);
//     qpslover.solve();
//     Eigen::MatrixXd result = qpslover.getResultP();

//     cout << "relsult: \n" << result << endl; 
// }

