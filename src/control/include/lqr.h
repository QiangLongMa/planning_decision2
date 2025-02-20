#pragma once
// #include"map.h"
#include "toolKits.h"


class lqr{
    private:

    public:
    int stateSpaceMatrix(Eigen::Matrix<double, 3, 3>& A, Eigen::Matrix<double, 3, 2>& B, const Eigen::VectorXd& car,  tool::controlParams cps,  double refDelta, const double refYaw);
    // int  calcTrackError(const tool::Car& car, const std::vector<tool::PathPoint>&  path, double& e, double& yaw, double& kappa, int& index);
    Eigen::Vector2d calcU(const Eigen::VectorXd& car, const Eigen::MatrixXd&  path, int& minIndex,
                                         const Eigen::Matrix<double, 3, 3>& A, const Eigen::Matrix<double, 3, 2>& B,
                                         const Eigen::Matrix<double, 3, 3>& Q, const Eigen::Matrix<double, 2, 2>& R);
 

    double lqrComputeCommand(double yaw, double delta_x ,double delta_y,double delta_yaw , 
            double curvature,double vel, double l, double dt,Eigen::MatrixXd& Q, Eigen::MatrixXd& R);

    Eigen::Vector2d calcU(const Eigen::VectorXd& car, const Eigen::MatrixXd&  path, 
                                            std::array<double,5> &Projection_point_message,
                                         const Eigen::Matrix<double, 3, 3>& A, const Eigen::Matrix<double, 3, 2>& B,
                                         const Eigen::Matrix<double, 3, 3>& Q, const Eigen::Matrix<double, 2, 2>& R);

};