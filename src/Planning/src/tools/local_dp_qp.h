#pragma once
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <vector>
#include "quintic_polynomial_curve1d.h"
#include "toolKits.h"
#include "QP.h"
#include "Speeddecisions.h"
#include <yaml-cpp/yaml.h>
#include "speed_.h"
#include "piecewise_jerk_speed_optimizer.h"
#include "localmath.h"
#include <iostream>
#include <fstream>
#include <string>
#include "ament_index_cpp/get_package_share_directory.hpp"

struct Node
{
  Node(const SlPoint &_sl_point)
      : sl_point(_sl_point), pre_node(nullptr), cost(std::numeric_limits<double>::max())
  {
  }
  SlPoint sl_point;
  Node *pre_node = nullptr; // 之前的节点
  double cost;
  double dl = 0;
  double ddl = 0;
  quintic_polynomial_curve5d curve;
};
struct Decisionflags
{
  bool ObstacleAvoidanceFlag = false; // 街道超车的标志true为正在避障， false 直线行驶
  bool DecelerateFlag = false;        // 减速停车标志
  bool DriveStraightLineFlag = false; // 直线行使标志
  bool Overtakinginlaneflag = false;  // 车道内超车的标志
  bool righttoleftlane = false;       // 超车完成 返回原车道的标志
};
class local_dp_qp
{
private:
  std::vector<std::vector<std::vector<Node>>> cost_table_;
  std::vector<std::vector<std::vector<SlPoint>>> sample_points_;
  Eigen::MatrixXd obstacles_;
  SlPoint vehicle_position;
  static constexpr double vehicle_length_ = 4.532;
  static constexpr double vehicle_width_ = 1.814;
  int all_distance;
  int one_distance;
  double first_dl = 0.0;
  double first_dll = 0.0;
  Eigen::MatrixXd path_param;                       // 路径参数 a b c d  a+b*s+c*s*s + d*s*s*s
  int SIZE_;                                        // 起点和终点之间点的数量
  std::vector<obses_sd> obsesSD_;                   // 障碍物的sd信息
  std::vector<Eigen::VectorXd> obs_limits_distance; // 在范围内的的障碍物
  Eigen::MatrixXd globalPath_;
  const double left_boundary = 3.5;
  const double right_boundary = -3.5;
  Eigen::MatrixXd lastOptTrajxy; // 本周期局部路径

  int CAR_INDEX_ = 0;
  double CAR_V;
  double CAR_A;
  double safetydistance; // 障碍物距离车辆最近的S
  bool advobsflag;       // 是否进行避障的标志

  double delta_s = 0.5;

  bool LANE_CHANGE_DECIDER = true; // 换道决策
  int BorrowingFlag;               // 借道标志 1 yes   0 no

  // bool StrightLineFlag=true; //为true表示在原车道行驶 且此时无碰撞，为false时在原车道行驶有碰撞

  double Start_l;
  double End_l;
  double Delta_l;
  double Target_v;
  double Target_l;
  bool ObstacleAvoidanceFlag_;
  bool DecelerateFlag_;
  bool DriveStraightLineFlag_; // 直线行使标志
  bool Overtakinginlaneflag_;  // 车道内超车的标志
  bool righttoleftlane_;       // 超车完成 返回原车道的标志

  bool FirstRunFlag; // 第一次运行的标志

  double path_l_cost;
  double path_dl_cost;
  double path_ddl_cost;
  // …在 loadyaml() 或其他适当的位置中：
  std::string pkg_share = ament_index_cpp::get_package_share_directory("planning");
  std::string yamllocate = pkg_share + "/YAML/param.yaml";
  std::vector<int> optinglobalindex; // 局部路径在全局路径中的编号
  YAML::Node config;

  bool ReducedDistanceFlag = false;    // 缩减距离的标志
  double Reducedistanceeachtime = 0.0; // 每次缩减的距离
  size_t Reducedistancesize = 0;       // 缩减几次

public:
  local_dp_qp();
  void loadyaml();
  void setPatam(double car_a, double car_v, double car_s, double car_l, double dl, double dll, const Eigen::MatrixXd &globalPath, int all_distances,
                int one_distances, int car_index, std::vector<obses_sd> obses_limit_SD,
                std::vector<Eigen::VectorXd> GlobalcoordinatesystemObsesLimit, double start_l, double end_l,
                double delta_l, double target_v, double traget_l, Decisionflags Decisionflags_, double obsmins, bool firstrunflag,
                bool ReducedDistanceFlag_, double Reducedistanceeachtime_, size_t Reducedistancesize_);
  void Setpraents(const std::vector<std::vector<std::vector<SlPoint>>> &sample_points);
  void sdtofrenet(const SlPoint SL, int &index, Eigen::Vector3d &tesianXY);
  void sdtofrenet(const SlPoint SL, int &index, Eigen::Vector2d &tesianXY);
  inline double Sigmoid(const double x);
  // 计算代价
  void CalculateCostTable();
  void CalculateCostAt(const int32_t s, const int32_t l, std::vector<std::vector<Node>> &single_cost);
  double CalculateAllObstacleCost(quintic_polynomial_curve5d curver, SlPoint pre_point, SlPoint cur_point);
  double CalculateReferenceLineCost(quintic_polynomial_curve5d curver, SlPoint pre_point, SlPoint cur_point);
  int FinalPath(Eigen::MatrixXd &optTrajxy, std::vector<Eigen::Vector4d> &frenet_path);
  void referencePath(const Eigen::MatrixXd &xy, const double v, Eigen::MatrixXd &path, std::vector<Eigen::Vector4d> &frenetpath);
  void referencePath(const Eigen::MatrixXd &xy, Eigen::MatrixXd &path, std::vector<Eigen::Vector4d> &frenetpath);
  int GetoptTrajxy(Eigen::MatrixXd &optTrajxy, std::vector<Eigen::Vector4d> &frenet_path);
  void GetSpeedLimit(Eigen::MatrixXd &optTrajxy, std::vector<Eigen::Vector4d> &optTrajsd, std::vector<double> &speedlimit_);
  double getmindistance(Eigen::MatrixXd &car_Fpoint, Eigen::Vector4d &obsxy);
  double getmindistance(Eigen::MatrixXd &car_Fpoint, obses_sd &obs_sd);
  void getspeeduselinearinterpolation(Eigen::MatrixXd &path);
  void getspeeduseST(Eigen::MatrixXd &path, std::vector<Eigen::Vector4d> &frenetpath);
  void getsamplepoints();

  void cartesianToFrenet(const Eigen::VectorXd &car, const Eigen::MatrixXd &path, frentPoint &carFrentPoint, int &minIndex);
  void findClosestPoint(const Eigen::VectorXd &car, const Eigen::MatrixXd &path, int &minIndex);
};
