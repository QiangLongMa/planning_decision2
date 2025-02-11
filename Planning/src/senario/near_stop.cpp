#include "near_stop.h"

// 通过初始化列表调用父类 Scenario 的构造函数
NearStop::NearStop(const Eigen::VectorXd &car,
    const Eigen::MatrixXd &globalPath,
    const Eigen::MatrixXd &obs_lidar)
: Scenario(car, globalPath, obs_lidar)
{
}

void NearStop::Straight()
{
}

void NearStop::AvoidObstacle()
{
}

void NearStop::Overtake()
{
}

void NearStop::DecelerateFollow()
{
}

void NearStop::ReturnRightLane()
{
}

void NearStop::MakeDecision()
{
}
