#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ros2_path_interfaces/srv/path_interfaces.hpp"
#include <Eigen/Eigen>
#include<math.h>
#include<cmath>
#include"discrete_points_reference_line_smoother.h"
// struct AnchorPoint
// {
//     double s;
//     double x;
//     double y;
//     double lateral_bound = 0.0;
//     double longitudinal_bound = 0.0;
//     // enforce smoother to strictly follow this reference point 强制平滑以严格遵循此参考点
//     bool enforced = false;
// };
class ReferenceLineSmoother
{
    private:
  
};

class ReferenceLineProvider
{
private:
    bool FLAGS_enable_smooth_reference_line=true; //是否进行平滑 
    double max_constraint_interval  =0.25;
    double longitudinal_boundary_bound =0.25;//纵向边界 
    double max_lateral_boundary_bound =0.5;//最大横向边界
    double min_lateral_boundary_bound = 0.2;//最小横向边界
    double curb_shift = 0.2;//与道路边缘的缓冲，若为实线，再增加一个buffer缓冲
    double lateral_buffer = 0.2;//道路边缘的缓冲
    double vehicle_width =1.872;
    DiscretePointsReferenceLineSmoother smooth_;
    
public:
    bool SmoothReferenceLine(Eigen::MatrixXd &RawglobalPath,Eigen::MatrixXd &NewglobalPath);
    void GetAnchorPoints(Eigen::MatrixXd &RawglobalPath,std::vector<AnchorPoint> *anchor_points);
    AnchorPoint GetAnchorPoint(Eigen::MatrixXd &RawglobalPath, Eigen::VectorXd col);

};

