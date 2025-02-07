#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "tools/toolKits.h"

// 新增状态枚举
enum class ScenarioState
{
    INIT,     // 第一次执行，初始化状态
    STRAIGHT, // 直行状态
    TURN,     // 转弯状态
    NEAR_STOP // 到达停止线附近
};

class ScenarioManager
{
public:
    ScenarioManager(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath, const Eigen::MatrixXd &obs_lidar);
    ~ScenarioManager() = default;

    // 初始化，传入车辆位置信息、全局路径信息及障碍物信息（摄像头、雷达

    // 状态更新函数，根据最新信息判断状态跳转
    ScenarioState Update();

    bool IsApproachingIntersection();//判断是否接近路口   

    bool IsApproachingStopLine();//判断是否接近停止线

    bool IsStraightForward();    // 判断是否直行状态

    bool ChangeFirstRun(); // 改变first_run状态

    // 使用内联函数返回车辆在frenet下的坐标和index
    inline tool::frentPoint GetFrentPoint() { return FrentPoint_; }
    // 使用内联函数返回车辆在全局坐标系下index
    inline int GetIndex() { return index_; }



    // 重置状态
    void Reset();

private:
    // 第一次运行
    bool first_run_ = true;
    // 当前状态
    ScenarioState state_;

    // 存储当前车辆状态
    Eigen::VectorXd current_car_;

    // 储存全局路径
    Eigen::MatrixXd current_globalPath_;

    // 存储障碍物信息（摄像头、雷达）
    Eigen::MatrixXd current_obs_lidar_;

    // 车辆在全局路径下的坐标点
    int index_ = 0;

    // 车辆在frenet下的坐标
    tool::frentPoint FrentPoint_;

};
