#pragma once
#include "scenario.h"
#include "localmath.h"
#include "QP.h"

class FirstRun : public Scenario
{
public:
    // 定义构造函数，同时传入父类构造函数需要的参数
    FirstRun(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath,
             const std::vector<obses_sd> &obses_limit_SD,
             const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
             const double &gpsA);
    // 只需要考虑直行
    void Straight();

    void MakeDecision() override;
    bool Process() override;

private:
    double delta_l;
    double target_v;
    double start_l;
    double end_l;
    double heading_time_;
    double ddl;
};
