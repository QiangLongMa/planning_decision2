#pragma once
#include<iostream>
#include<Eigen/Core>
#include<Eigen/Dense>
#include <Eigen/Eigen>
#include<vector>
/*************速度计算  simple***************/
class SpeedDecisions 
{

private:
    double T= 2;
public:
    void GetSpeed(double car_s,double start_v,double end_v,Eigen::MatrixXd &optTrajxy,double safetydistance,bool flag);
    void closeobsgetspeed(Eigen::MatrixXd &optTrajxy);
};






