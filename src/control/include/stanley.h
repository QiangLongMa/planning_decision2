#pragma once
// #include "map.h"
#include "toolKits.h"

class stanley{
    private:
        double delta_y = 0.0;
        double delta_yaw = 0.0;
        double delta_kappa = 0.0;
        double delta_e = 0.0;
        double delta_Steer = 0.0;
        double k_d = 10;
        double k_yaw = 10;
        double k_kappa = 10;
    public:
        stanley( double Kd, double Kyaw, double Kkappa);
        double stanleyControl(const Eigen::VectorXd& car, const Eigen::Matrix<double, 5, Eigen::Dynamic>&  path, tool::frentPoint carFrentPoint, int& minIndex);
};