#pragma once
#include <iostream>
#include <Eigen/Dense>
#include "local_dp_qp.h"

class Scenario{
    public:
    Scenario ();
    virtual ~Scenario() = default;

    virtual bool Init();

    virtual bool IsTransferable(const Scenario& other_scenario, const Eigen::MatrixXd &obs_cam, const Eigen::MatrixXd &obs_lidar,const Eigen::VectorXd &car,const Eigen::MatrixXd& globalPath)
    {
        return false;
    };

    virtual Decisionflags Process();

    virtual bool Exit();

    virtual bool Enter();



    private:
    Decisionflags decisionflags_;


};
