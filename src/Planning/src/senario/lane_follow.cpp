#include "lane_follow.h"

// 通过初始化列表调用父类 Scenario 的构造函数
LaneFollowScenario::LaneFollowScenario(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath,
                                       const std::vector<obses_sd> &obses_limit_SD,
                                       const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
                                       const double &gpsA)  
    : Scenario(car, globalPath, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA)
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

bool LaneFollowScenario::Process()
{
    return false;
}
