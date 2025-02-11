#include "scenario_tools.h"

namespace senarioTools{
    /******************功能函数**************************/

void findClosestPoint(const double& x, const double& y,const Eigen::MatrixXd& globalPath, int& carIndex){
    int startIndex = std::max(carIndex - 10,0);
    int endIndex = std::min(carIndex + 120, static_cast<int>(globalPath.cols()));
    double distance,d_min = std::numeric_limits<double>::max();
    for (int i= startIndex; i < endIndex; ++i){
        distance = (std::pow(globalPath(0, i) - x, 2) + std::pow(globalPath(1, i) - y, 2));
        if (distance < d_min) {
            carIndex = i;
            d_min = distance;	
        }
    } 
}

void cartofrenet(const Eigen::VectorXd& CAR, const Eigen::MatrixXd& path, int& carIndex, tool::frentPoint &carFrent) {
    const double dx = CAR(0) - path(0, carIndex);
    const double dy = CAR(1) - path(1, carIndex);
    const double theta = path(3, carIndex);
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);
    double ref_s = path(6, carIndex);
    double path_s = dx * cos_theta + dy * sin_theta + ref_s;
    const double cross = cos_theta * dy - sin_theta * dx;
    carFrent.d = std::copysign(cross, cross);
    carFrent.s = std::abs(path_s);
}



}