#pragma once
#include "scenario.h"


class LaneFollowScenario : public Scenario
{
public:
    // 定义构造函数，同时传入父类构造函数需要的参数
    LaneFollowScenario(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath, const Eigen::MatrixXd &obs_lidar);
    void Straight() override;
    void AvoidObstacle() override;
    void Overtake() override;
    void DecelerateFollow() override;
    void ReturnRightLane() override;
    void MakeDecision() override;

private:
};
