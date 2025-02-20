#pragma once
#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include "toolKits.h"

namespace  local{
    int getOptimalLocalTraj(const Eigen::VectorXd& carRealPosition, const Eigen::MatrixXd& globalPath, Eigen::MatrixXd& obs1, tool::controlParams& cps, Eigen::MatrixXd& cartesianTrajt, Eigen::MatrixXd& frenetTraj, int& index, bool fre);
    int showCartesianLocalTrajs(const std::vector<Eigen::MatrixXd>& frenetLocalTrajs, const Eigen::MatrixXd& globalPath, const int& optimal, const int& initMinIndex, const tool::controlParams cps, std::vector<Eigen::MatrixXd>& allCartesianXY);
    int genCartesianLocalTrajs( std::vector<Eigen::MatrixXd>& frenetLocalTrajs, const Eigen::MatrixXd& globalPath, const int& optimal, const int& initMinIndex, const tool::controlParams cps, Eigen::MatrixXd& cartesianXY);
    int genFrenetLocalTrajs(const Eigen::VectorXd& carRealPosition, const Eigen::MatrixXd& globalPath, const Eigen::MatrixXd& obstructs, tool::controlParams& cps, std::vector<Eigen::MatrixXd>& localTrajs, int& minIndex);
    double evalfun(const tool::controlParams& cps, const tool::states& xyStates, const Eigen::MatrixXd& obstructs, Eigen::Vector4d& Ai, Eigen::Vector4d& Bi,  Eigen::VectorXd& cost_one_Traj, Eigen::MatrixXd& localTraj);
    void findClosestPoint(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, int& minIndex);
    void carfindClosestPoint(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, int& minIndex);
    void obsfindClosestPoint(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, int& minIndex, const int& carIndex);
    int cartesian2Frenet(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, tool::frentPoint& carFrentPoint, int& index,int &carIndex, const int flag);
    int frenet2Cartesian(const Eigen::MatrixXd& globalPath, const double& s, const double& d, const int& minIndex, Eigen::Vector2d& cartesianXY);
    Eigen::Vector4d cubicCoffec(double tT, double pStart, double vStart, double pEnd, double vEnd);
    int referencePath(const Eigen::MatrixXd& xy, const double v, Eigen::MatrixXd& reTrajt);
}