#pragma once
#include <iostream>
#include <Eigen/Dense>

#include "local_dp_qp.h"
#include "toolKits.h"
#include "scenario_tools.h"

class Scenario
{
public:
    Scenario(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath, const Eigen::MatrixXd &obs_lidar);
    virtual ~Scenario() = default;

    // 直行决策
    virtual void Straight() = 0;

    // 避障决策
    virtual void AvoidObstacle() = 0;

    // 超车决策
    virtual void Overtake() = 0;

    // 减速跟车决策
    virtual void DecelerateFollow() = 0;

    // 返回原车道决策
    virtual void ReturnRightLane() = 0;

    // 给出决策makedecision的函数
    virtual void MakeDecision() = 0;

    // 执行函数
    bool Process();

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
    Eigen::MatrixXd obs_lidar_;

    // 车辆在局部路径下的坐标点
    int index = 0;

    //是否找到合适的局部路径 
    bool find_local_path_ = false;

};
