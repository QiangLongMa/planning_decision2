#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "tools/local_dp_qp.h"



class Scenario
{
public:
    Scenario(){};
    virtual ~Scenario() = default;

    // 直行决策
    virtual void Straight() = 0;

    // 避障决策
    virtual void AvoidObstacle() = 0;

    // 超车决策
    virtual void Overtake() = 0;

    // 减速跟车决策
    virtual void DecelerateFollow() = 0;

    // 紧急停止决策
    virtual void EmergencyStop() = 0;

    virtual Decisionflags Process();

    // 工厂函数，根据不同类型返回不同的子类实例
    static std::unique_ptr<Scenario> CreateScenario(int type);



private:
    Decisionflags Decisionflags_;
    local_dp_qp LOCAL_;

};
