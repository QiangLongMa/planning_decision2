#pragma once
#include <iostream>
#include <Eigen/Dense>

#include "local_dp_qp.h"
#include "toolKits.h"
#include "scenario_tools.h"

class Scenario
{
public:
    Scenario(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath,
             const std::vector<obses_sd> &obses_limit_SD,
             const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
             const double &gpsA);
    virtual ~Scenario() = default;



    // 给出决策makedecision的函数
    virtual void MakeDecision() = 0;

    // 执行函数
    virtual bool Process() = 0;

    // 获取当前系统时间
    double Time();

    // 重置决策flags
    void RestFlags();

    // 获取本周期的局部路径optTrajxy
    inline Eigen::MatrixXd getlocalpath() const { return optTrajxy; }

protected:
    Decisionflags Decisionflags_;
    local_dp_qp LOCAL_;
    Eigen::MatrixXd optTrajxy;                  // 本周期局部路径
    std::vector<Eigen::Vector4d> optTrajsd;     // 上周期局部路径
    Eigen::MatrixXd lastOptTrajxy;              // 本周期局部路径
    std::vector<Eigen::Vector4d> lastOptTrajsd; // 本周期局部路径

    // 存储当前车辆状态
    Eigen::VectorXd car_;

    // 储存全局路径
    Eigen::MatrixXd globalPath;

    // 存储障碍物信息（摄像头、雷达）
    std::vector<obses_sd> obses_limit_SD;
    std::vector<Eigen::VectorXd> GlobalcoordinatesystemObsesLimit;
    double gpsA_;
    

    // 车辆在局部路径下的坐标点
    int index = 0;

    // 是否找到合适的局部路径
    bool find_local_path_ = false;
};
