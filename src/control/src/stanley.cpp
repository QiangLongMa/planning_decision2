#include "stanley.h"

stanley::stanley( double Kd, double Kyaw, double Kkappa):k_d(Kd),  k_yaw(Kyaw), k_kappa(Kkappa){
        // double k_d = Kd;
        // double k_yaw = Kyaw;
        // double k_kappa = Kkappa;
}
double stanley::stanleyControl(const Eigen::VectorXd& car, const Eigen::Matrix<double, 5, Eigen::Dynamic>&  path, tool::frentPoint carFrentPoint, int& minIndex){
        delta_yaw =  sin(path(3, minIndex) - tool::d2r(car(3)));
        delta_e = atan2(k_d * carFrentPoint.d, car(4));
        delta_kappa = atan2(car(4) * path(4, minIndex), 1);
        delta_Steer =  tool::r2d(delta_e) - k_yaw *delta_yaw - k_kappa * tool::r2d(delta_kappa);
        return delta_Steer;
}
