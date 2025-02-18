#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "toolKits.h"

namespace senarioTools{

    void findClosestPoint(const double& x, const double& y,const Eigen::MatrixXd& globalPath, int& carIndex);
    void cartofrenet(const Eigen::VectorXd& CAR, const Eigen::MatrixXd& path, int& carIndex, tool::frentPoint &carFrent);
    

}