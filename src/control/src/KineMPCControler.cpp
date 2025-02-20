//
// Created by yuanyancheng on 2020/11/11.
//
#include "KineMPCControler.h"
#include <cmath>
#include "Eigen/LU"
#include <iostream>
using Matrix = Eigen::MatrixXd;
KineMPCController::KineMPCController()
{
    double thetar = 0;
    double vr = 0;
    double deltar = 0;

    //wheelbase_ = 2.647;
    // Matrix init operations.
    matrix_ad_ = Matrix::Identity(basic_state_size_, basic_state_size_);
    // matrix_ad_(0, 0) = 1.0;
    // // matrix_ad_(0, 2) = -vr*sin(thetar)*ts_;
    // matrix_ad_(1, 1) = 1.0;
    // // matrix_ad_(1, 2) = vr*cos(thetar)*ts_;
    // matrix_ad_(2, 2) = 1.0;
    matrix_bd_ = Matrix::Zero(basic_state_size_, controls_);
    // matrix_bd_(0, 0) = cos(thetar)*ts_;
    // matrix_bd_(1, 0) = sin(thetar)*ts_;
    // matrix_bd_(2, 0) = tan(deltar)*ts_/wheelbase_;
    // matrix_bd_(2, 1) = vr*ts_/wheelbase_/cos(deltar)/cos(deltar);
    matrix_cd_ = Matrix::Zero(basic_state_size_, 1);
    matrix_state_ = Matrix::Zero(basic_state_size_, 1);
    matrix_r_ = Matrix::Zero(controls_, controls_);
    matrix_q_ = Matrix::Identity(basic_state_size_, basic_state_size_);
    double qarray[] = {100, 100, 100};
    for (int i = 0; i < 3; ++i) {
        matrix_q_(i, i) = qarray[i];
    }
    matrix_r_(1, 1) = 1;
}
//计算误差 
void KineMPCController::ComputeStateErrors(std::tuple<double,double,double,double> &state,
    std::tuple<double,double,double,double> &reference_point) {
    // x y theta 
    //reference_point = traj->QueryPathPointAtDistance(state->speed*t);
    double xTarget = std::get<0>(reference_point);
    double yTarget = std::get<1>(reference_point);
    //std::cout<<state->speed*t<<","<<reference_point.x<<","<<reference_point.y<<std::endl;
    x_error   = std::get<0>(state) - xTarget;
    y_error   = std::get<1>(state) - yTarget;
    phi_error =  control_math::NormalizeAngle(std::get<2>(state)  - std::get<2>(reference_point));
    std::cout<<"x_error: "<<x_error<<std::endl;
    std::cout<<"y_error: "<<y_error<<std::endl;
    std::cout<<"phi_error: "<<phi_error<<std::endl;
}

//更新状态
void KineMPCController::UpdateStateAnalyticalMatching() {
    matrix_state_(0, 0) = x_error;//x
    matrix_state_(1, 0) = y_error;//y
    matrix_state_(2, 0) = phi_error;//theta 
    std::cout<<"matrix_state_(0, 0) : "<<x_error<<std::endl;
    std::cout<<"matrix_state_(1, 0) : "<<y_error<<std::endl;
    std::cout<<"matrix_state_(2, 0) : "<<phi_error<<std::endl;
}

//更新A B
void KineMPCController::UpdateMatrix(std::tuple<double,double,double,double> &state,
    std::tuple<double,double,double,double> &reference_point) {
    double vr = 3;
    std::cout<<"std::get<3>(reference_point) : "<<std::get<3>(reference_point)<<std::endl;
    double deltar = atan2(std::get<3>(reference_point) * wheelbase_, 1);           //wheelbase_ 轴距  Curvature 曲率 
    std::cout<<"deltar : "<<deltar<<std::endl;
    double thetar = control_math::NormalizeAngle(std::get<2>(reference_point));
    std::cout<<"thetar : "<<thetar<<std::endl;
    matrix_ad_(0, 2) = -vr * sin(thetar) * ts_;
    matrix_ad_(1, 2) = vr * cos(thetar) * ts_;
    matrix_bd_(0, 0) = cos(thetar) * ts_;
    matrix_bd_(1, 0) = sin(thetar) * ts_;
    matrix_bd_(2, 0) = tan(deltar) * ts_/wheelbase_;
    matrix_bd_(2, 1) = vr * ts_ / wheelbase_ / cos(deltar) / cos(deltar);
}

//reference_point : x y theta Curvature 
bool KineMPCController::ComputeControlCommand(std::tuple<double,double,double,double> &state,
    std::tuple<double,double,double,double> &reference_point, std::vector<double> &cmd){
    ComputeStateErrors(state, reference_point);
    UpdateStateAnalyticalMatching();
    UpdateMatrix(state, reference_point);
    Eigen::MatrixXd control_matrix(controls_, 1);
    control_matrix << 0, 0;
    Eigen::MatrixXd reference_state(basic_state_size_, 1);
    reference_state << 0, 0, 0;
    std::vector<Eigen::MatrixXd> reference(horizon_, reference_state);//步长 和状态量 
    std::vector<Eigen::MatrixXd> control(horizon_, control_matrix);
    constexpr double wheel_single_direction_max_degree_ = 35 / 180 * M_PI;//车轮转角限制 0.610
    constexpr double max_acceleration_ = 5;
    constexpr double max_deceleration_ = -5;

    Eigen::MatrixXd lower_bound(controls_, 1);
    lower_bound << -wheel_single_direction_max_degree_, max_deceleration_;
    Eigen::MatrixXd upper_bound(controls_, 1);
    upper_bound << wheel_single_direction_max_degree_, max_acceleration_;

    const double max = std::numeric_limits<double>::max();
    Eigen::MatrixXd lower_state_bound(basic_state_size_, 1);
    Eigen::MatrixXd upper_state_bound(basic_state_size_, 1);
    lower_state_bound << -1.0 * max, -1.0 * max, -1.0 * M_PI;
    upper_state_bound<<  max,  max,  M_PI;
    std::vector<double> control_cmd(controls_, 1);

    //求解MPC    A B C Q R 下边界 上边界 当前误差量  步长和状态量 阈值 迭代次数 控制 
    MpcOsqp mpx_osqp(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_,
        matrix_state_, lower_bound, upper_bound, lower_state_bound,
        upper_state_bound, reference_state, mpc_max_iteration_, horizon_,
        mpc_eps_);
    if (!mpx_osqp.Solve(&control_cmd)) {
        std::cout<<"MPC OSQP solver failed"<<std::endl;
        return false;
    } else {
        double steer = control_cmd.at(0);
        double acc = control_cmd.at(1);
        cmd.push_back(steer);
        cmd.push_back(acc);
        return true;
    }
}
    
       

   
    