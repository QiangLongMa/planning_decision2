#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace localMath
{
  struct SpeedPoint
  {
    double x;
    double y;
    double v;
    double theta;
    double dkappa; // 曲率
    double s;
    double a;
    double t;
    double Absolute_time; // 绝对时间
    // SpeedPoint(double s_ ,double t_):s(s_), t(t_){}
  };
  struct SpeedLimit
  {
    double s;
    double v;
    SpeedLimit() {}
    SpeedLimit(double s_, double v_) : s(s_), v(v_) {}
  };

  struct SpeedDkappa
  {
    double s;
    double v;
    double dkappa;
  };
  double NormalizeAngle(double angle);
  double slerp(const double a0, const double t0, const double a1, const double t1,
               const double t);

  double lerp(const double &x0, const double t0, const double &x1, const double t1,
              const double t);

  double InterpolateUsingLinearApproximation(double s0, double s1, double s, double dkappa0, double dkappa1);

  bool EvaluateByTime(const double t, std::vector<SpeedPoint> &speed_data, SpeedPoint &speed_point);

  double GetSpeedLimitByS(std::vector<SpeedLimit> &speedlimit, double s);

  double GetCuriseSpeedByS(std::vector<SpeedDkappa> &curise_speed, double s);

  void EvaluateByS(std::vector<std::vector<double>> &vec, const double s, SpeedPoint &speed_point);
  void GetsldlBys(std::vector<Eigen::Vector4d> &localpath, double path_s, double &start_point_s,
                  double &start_point_l, double &start_point_dl, double &start_point_ddl);
  double Caldll(double l, double dl, double ref_dkappa, double ref_kappa, double delta, double ptr_kappa);
} // namespace localMtah
