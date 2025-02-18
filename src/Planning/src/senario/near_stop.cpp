#include "near_stop.h"

// 通过初始化列表调用父类 Scenario 的构造函数
NearStop::NearStop(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath,
                   const std::vector<obses_sd> &obses_limit_SD,
                   const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
                   const double &gpsA)
    : Scenario(car, globalPath, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA)
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

bool NearStop::Process()
{
    return false;
}
