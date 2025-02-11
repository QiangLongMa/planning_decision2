#include <iostream>
#include <cmath>
#include <limits>
#include <vector>
#include <memory>
#include "dp_st_cost_.h"
#include <algorithm>
#include <limits>
#include "localmath.h"

class StGraphPoint
{
private:
  STPoint point_;
  const StGraphPoint *pre_point_ = nullptr;
  std::uint32_t index_s_ = 0;
  std::uint32_t index_t_ = 0;
  double optimal_speed_ = 0.0;
  double reference_cost_ = 0.0;
  double obstacle_cost_ = 0.0;
  double spatial_potential_cost_ = 0.0;
  double total_cost_ = std::numeric_limits<double>::infinity();

public:
  std::uint32_t index_s() const;
  std::uint32_t index_t() const;

  const STPoint &point() const;
  const StGraphPoint *pre_point() const;

  double reference_cost() const;
  double obstacle_cost() const;
  double spatial_potential_cost() const;
  double total_cost() const;

  void Init(const std::uint32_t index_t, const std::uint32_t index_s,
            const STPoint &st_point);
  // given reference speed profile, reach the cost, including position
  void SetReferenceCost(const double reference_cost);
  // given obstacle info, get the cost;
  void SetObstacleCost(const double obs_cost);
  // given potential cost for minimal time traversal 考虑到最小时间遍历的潜在成本
  void SetSpatialPotentialCost(const double spatial_potential_cost);
  // total cost
  void SetTotalCost(const double total_cost);
  void SetPrePoint(const StGraphPoint &pre_point);
  double GetOptimalSpeed() const;
  void SetOptimalSpeed(const double optimal_speed);
  StGraphPoint();
};
// defined for cyber task
struct StGraphMessage
{
  StGraphMessage(const uint32_t c_, const int32_t r_) : c(c_), r(r_) {}
  uint32_t c;
  uint32_t r;
};

class speed_
{
private:
  static constexpr double kDoubleEpsilon = 1.0e-6;
  dp_st_cost_ dp_st_cost;
  int total_length_t_ = 20; // 车辆的速度很慢 所以要增加对应的时间
  int unit_t_ = 1.0;
  double dense_unit_s_, dimension_t_;

  double max_deceleration_ = -2.5; // 最大减速度
  double max_acceleration_ = 2;    // 最大加速度
  // cost_table_[t][s]
  std::vector<std::vector<StGraphPoint>> cost_table_;
  std::vector<double> spatial_distance_by_index_;
  std::vector<double> speed_limit_by_index_;
  double total_s_;
  int dimension_s_;

  double sparse_length_s = 0.5;
  double init_point_v; // 初始的速度
  double init_point_a;
  double spatial_potential_penalty = 1e5;    // 终点权重
  double speed_limit;                        // 速度限制 后面应该要根据全局路径的曲率进行选择
  std::vector<localMath::SpeedDkappa> car_cruise_speed; // 巡航速度 参考的速度
  std::vector<double> speed_limit_list;
  std::vector<double> speed_dkppa;

public:
  void InitCostTable(double total_length_s_);
  void InitSpeedLimitLookUp();
  void CalculateTotalCost();
  void CalculateCostAt(const std::shared_ptr<StGraphMessage> &msg);

  double CalculateEdgeCost(const STPoint &first, const STPoint &second,
                           const STPoint &third, const STPoint &forth,
                           const double speed_limit, const double cruise_speed);

  double CalculateEdgeCostForSecondCol(const uint32_t row,
                                       const double speed_limit,
                                       const double cruise_speed);

  double CalculateEdgeCostForThirdCol(const uint32_t curr_row,
                                      const uint32_t pre_row,
                                      const double speed_limit,
                                      const double cruise_speed);

  void GetObstacleCost(const StGraphPoint const_cr);
  double GetPatialPotentialCost(const StGraphPoint const_cr);

  void GetRowRange(const StGraphPoint &point, size_t *next_highest_row, size_t *next_lowest_row);

  bool RetrieveSpeedProfile(std::vector<localMath::SpeedPoint> &speed_data);

  speed_();

  bool Search(double total_length_s_, std::vector<localMath::SpeedPoint> &speed_data, double init_point_v_, double init_point_a_, std::vector<localMath::SpeedDkappa> cruise_speed_,
              std::vector<double> speed_limit_list_, std::vector<double> &speed_dkppa_, YAML::Node &config_);
};
