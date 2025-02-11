#include "approaching_intersection.h"

// 通过初始化列表调用父类 Scenario 的构造函数
ApproachingIntersection::ApproachingIntersection(const Eigen::VectorXd &car,
                                       const Eigen::MatrixXd &globalPath,
                                       const Eigen::MatrixXd &obs_lidar)
    : Scenario(car, globalPath, obs_lidar)
{
}

void ApproachingIntersection::Straight()
{
}

void ApproachingIntersection::AvoidObstacle()
{
}

void ApproachingIntersection::Overtake()
{
}

void ApproachingIntersection::DecelerateFollow()
{
}

void ApproachingIntersection::ReturnRightLane()
{
}

void ApproachingIntersection::MakeDecision()
{
}
