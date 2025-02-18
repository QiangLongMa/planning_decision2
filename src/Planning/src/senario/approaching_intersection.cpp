#include "approaching_intersection.h"

// 通过初始化列表调用父类 Scenario 的构造函数
ApproachingIntersection::ApproachingIntersection(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath,
                                                 const std::vector<obses_sd> &obses_limit_SD,
                                                 const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
                                                 const double &gpsA)
    : Scenario(car, globalPath, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA)
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

bool ApproachingIntersection::Process()
{
    return false;
}
