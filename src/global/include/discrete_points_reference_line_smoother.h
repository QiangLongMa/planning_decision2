#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ros2_path_interfaces/srv/path_interfaces.hpp"
#include <Eigen/Eigen>
#include<math.h>
#include<cmath>
#include"fem_pos_deviation_osqp_interface.h"
struct AnchorPoint
{
    double s;
    double x;
    double y;
    double lateral_bound = 0.0;
    double longitudinal_bound = 0.0;
    // enforce smoother to strictly follow this reference point 强制平滑以严格遵循此参考点
    bool enforced = false;
};


class DiscretePointsReferenceLineSmoother
{

private:
    std::vector<AnchorPoint> anchor_points_;
    double zero_x_ = 0.0;
    double zero_y_ = 0.0;


    //qp param 
    double weight_fem_pos_deviation= 1e10;
    double weight_ref_deviation= 1.0;
    double weight_path_length= 1.0;
    bool apply_curvature_constraint= false;
    int max_iter= 500;
    double time_limit= 0.0;
    bool verbose= false;
    bool scaled_termination= true;
    bool warm_start= true;



public:
    void SetAnchorPoints(const std::vector<AnchorPoint>& anchor_points);
    bool Smooth(const Eigen::MatrixXd& raw_reference_line,Eigen::MatrixXd &smoothed_reference_line);
    void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);
    void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);
    
    bool FemPosSmooth(const std::vector<std::pair<double, double>>& raw_point2d,const std::vector<double>& bounds,std::vector<std::pair<double, double>>* ptr_smoothed_point2d);
    bool Solve(const std::vector<std::pair<double, double>>& raw_point2d,const std::vector<double>& bounds,
                std::vector<double> *opt_x,std::vector<double> *opt_y);

    void referencePath(const Eigen::MatrixXd& xy,Eigen::MatrixXd &path);
};

