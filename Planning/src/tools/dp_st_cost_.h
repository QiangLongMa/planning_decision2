#pragma once
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "yaml-cpp/yaml.h"

struct STPoint
{
    double t;
    double s;
    STPoint() : t(0.0), s(0.0) {} // 默认构造函数
    STPoint(const double t_, const double s_) : t(t_), s(s_) {}
};

constexpr double kInf = std::numeric_limits<double>::infinity();
class dp_st_cost_
{
private:
    int unit_t_ = 1.0;
    double max_acceleration;
    double max_deceleration;
    double max_jerk;
    double min_jerk;
    double accel_penalty;
    double decel_penalty;
    double exceed_speed_penalty;
    double default_speed_cost;
    double low_speed_penalty;
    double reference_speed_penalty;
    double positive_jerk_coeff;
    double negative_jerk_coeff;
    double accel_weight;
    double jerk_weight;
    std::array<double, 500> accel_cost_;
    std::array<double, 500> jerk_cost_;
    // std::string yamllocate="src/local/src/param.yaml";

public:
    dp_st_cost_(/* args */);
    ~dp_st_cost_();
    double GetSpeedCost(const STPoint &first, const STPoint &second,
                        const double speed_limit,
                        const double cruise_speed) const;
    // 计算加 accel_cost
    double GetAccelCost(const double accel);
    // 计算 jerk_cost
    double JerkCost(const double jerk);

    double GetAccelCostByTwoPoints(const double pre_speed, const STPoint &first,
                                   const STPoint &second);
    double GetAccelCostByThreePoints(const STPoint &first, const STPoint &second,
                                     const STPoint &third);

    double GetJerkCostByTwoPoints(const double pre_speed, const double pre_acc,
                                  const STPoint &pre_point,
                                  const STPoint &curr_point);
    double GetJerkCostByThreePoints(const double first_speed,
                                    const STPoint &first_point,
                                    const STPoint &second_point,
                                    const STPoint &third_point);
    double GetJerkCostByFourPoints(const STPoint &first,
                                   const STPoint &second,
                                   const STPoint &third,
                                   const STPoint &fourth);
    void loadyaml(YAML::Node &config);
};
