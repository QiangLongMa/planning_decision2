#pragma once
#include "scenario.h"

class LaneFollowScenario : public Scenario
{
public:
    // 定义构造函数，同时传入父类构造函数需要的参数
    LaneFollowScenario(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath,
                       const std::vector<obses_sd> &obses_limit_SD,
                       const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
                       const double &gpsA, const double indexinglobalpath);
    bool Straight();
    bool AvoidObstacle();
    void Overtake();
    bool DecelerateFollow();
    void ReturnRightLane();

    void MakeDecision() override;
    bool Process() override;

private:
    double delta_l;
    double target_v;
    double start_l;
    double end_l;
    double target_l;
    double heading_time_;
    double ddl;
    double dl;
    double distance_threshold = 9; //停车距离阈值 自车长度 + 前方停车距离 (和前方车辆的距离)
    bool flag;
    double local_start_s;
    double local_start_l;
    double speed;
};
