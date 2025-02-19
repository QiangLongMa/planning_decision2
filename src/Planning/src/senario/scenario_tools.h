#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "toolKits.h"
#include "Calobs.h"
#include "localmath.h"

namespace senarioTools{
    constexpr double vehicle_length_ = 5.136;
    constexpr double vehicle_width_ = 2.016;
    void findClosestPoint(const double& x, const double& y,const Eigen::MatrixXd& globalPath, int& carIndex);
    void cartofrenet(const Eigen::VectorXd& CAR, const Eigen::MatrixXd& path, int& carIndex, frentPoint &carFrent);
    std::pair<bool, double> Pathplanningduringdeceleration(Eigen::MatrixXd& path, std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimitinlocal_);
    void getclose_s(Eigen::MatrixXd &path, std::vector<Eigen::VectorXd> &obses, double &min_s, bool &ischeck);
    std::pair<double, double> ComputePositionProjection(
            const double x, const double y, const double ref_x, const double ref_y
            , const double theta, const double ref_s);
  //计算L的二阶导数
    std::array<double, 6> ComputeReinitStitchingTrajectory (int localcloseindex,
        int globalcloseindex,frentPoint &FrentPoint, Eigen::VectorXd &vehicle_state,
        Eigen::MatrixXd &localpath, int indexinglobalpath, Eigen::MatrixXd &globalPath);
    
    std::array<double, 6> Decidestartsl(frentPoint &FrentPoint,int position_matched_index,
                        int globalcloseindex, Eigen::MatrixXd &localpath,
                        Eigen::MatrixXd &globalPath, Eigen::VectorXd &vehicle_state, 
                        std::vector<Eigen::Vector4d>&optTrajsd);
}