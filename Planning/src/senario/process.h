#pragma once
#include<Eigen/Dense>

#include <Eigen/Dense>

class Frame {
public:
    // 构造函数
    Frame(const Eigen::MatrixXd& obs_lidar, const Eigen::VectorXd& car)
        : obs_lidar_(obs_lidar), car_(car) {}

    // 成员变量
    Eigen::MatrixXd obs_lidar_;
    Eigen::VectorXd car_;
};