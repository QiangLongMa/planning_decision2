#pragma once
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include "osqp/osqp.h"
#include "piecewise_jerk_problem.h"
#include "piecewise_jerk_path_problem.h"
#include "piecewise_jerk_trajectory1d.h"
#include <chrono>

using namespace std;
struct SlPoint
{
  double s;
  double l;
  int index; // 在全局路径中的编号
};

struct obses_sd : SlPoint
{
  SlPoint point1;
  SlPoint point2;
  SlPoint point3;
  SlPoint point4;
  SlPoint centre_points;
  double min_s;
  double max_s;
  double length;
  double wigth;
};

struct FrenetFramePoint
{
  double s;
  double l;
  double dl;
  double ddl;
};

class QP
{
private:
  double left_bound = 2.9; // 左右边界
  double right_bound = -2.9;
  double car_wigth = 1.8; // 车辆的宽度
  std::vector<std::pair<double, double>> hdmap_bound_;
  std::vector<std::pair<double, double>> static_obstacle_bound_; // 静态障碍物的边界 上边界和边界
  std::vector<std::pair<double, double>> l_limit;                // l
  double max_steer_angle = 33.5 / 180 * M_PI;
  double max_steer_angle_rate = 6.98131700798;
  double steer_ratio = 16.5;
  double wheel_base = 2.65;
  Eigen::MatrixXd globalPath_;
  double delta_s = 0.5;

public:
  QP();
  ~QP();
  void CalculateHDMapBound(std::vector<Eigen::Vector4d> &frenet_path); // 计算道路边界  frenet_path 局部路径的信息
  void MapStaticObstacleWithDecision(double car_s, double car_l, const std::vector<obses_sd> &obstacle, std::vector<Eigen::Vector4d> &frenet_path);
  void CalculateLLimit(std::vector<Eigen::Vector4d> &frenet_path, const std::vector<obses_sd> &obstacle);

  void CalculateLLimit();

  bool Process(std::pair<std::array<double, 3>, std::array<double, 3>> inint_start_point, const std::vector<obses_sd> &obstacle,
               std::vector<Eigen::Vector4d> &frenet_path, const Eigen::MatrixXd &globalPath, Eigen::MatrixXd &xy_list_qp,
               bool changelaneflag, double leftbound, double rightboun);

  bool OptimizePath(
      const std::array<double, 3> &init_state,
      const std::array<double, 3> &end_state,
      std::vector<double> path_reference_l_ref, const size_t path_reference_size,
      const double delta_s, const bool is_valid_path_reference,
      const std::vector<std::pair<double, double>> &lat_boundaries,
      const std::vector<std::pair<double, double>> &ddl_bounds,
      const std::array<double, 5> &w, const int max_iter, std::vector<double> *x,
      std::vector<double> *dx, std::vector<double> *ddx);
  double GaussianWeighting(const double x, const double peak_weighting, const double peak_weighting_x);
  inline double Gaussian(const double u, const double std, const double x);
  inline double EstimateJerkBoundary(const double vehicle_speed, const double axis_distance, const double max_yaw_rate);

  std::vector<FrenetFramePoint> ToPiecewiseJerkPath(const std::vector<double> &x, const std::vector<double> &dx,
                                                    const std::vector<double> &ddx, const double delta_s, const double start_s);
  bool PointInRect(Eigen::Vector2d &vec12, Eigen::Vector2d &vec23, Eigen::Vector2d &vec34, Eigen::Vector2d &vec41, Eigen::Vector2d &vec1_p1, Eigen::Vector2d &vec1_p2, Eigen::Vector2d &vec1_p3, Eigen::Vector2d &vec1_p4);
  bool IsPointInRect(std::vector<Eigen::Vector4d> &frenet_path, const std::vector<obses_sd> &obstacle);
};
