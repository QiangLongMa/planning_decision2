#include "lane_follow.h"

// 通过初始化列表调用父类 Scenario 的构造函数
LaneFollowScenario::LaneFollowScenario(const Eigen::VectorXd &car,
                                       const Eigen::MatrixXd &globalPath,
                                       const Eigen::MatrixXd &obs_lidar)
    : Scenario(car, globalPath, obs_lidar)
{
}

void LaneFollowScenario::Straight()
{
}

void LaneFollowScenario::AvoidObstacle()
{
}

void LaneFollowScenario::Overtake()
{
}

void LaneFollowScenario::DecelerateFollow()
{
}

void LaneFollowScenario::ReturnRightLane()
{
}

void LaneFollowScenario::MakeDecision()
{
}
