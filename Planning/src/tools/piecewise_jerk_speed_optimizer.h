#include <tuple>
#include <utility>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "localmath.h"
#include "piecewise_jerk_speed_problem.h"
#include "yaml-cpp/yaml.h"
#include "localmath.h"
class PiecewiseJerkSpeedOptimizer
{
private:
    std::vector<SpeedPoint> reference_speed_data;
    double acc_weight;
    double jerk_weigth;
    double max_deceleration;
    double max_acceleration;
    double longitudinal_jerk_lower_bound;
    double longitudinal_jerk_upper_bound;
    double ref_v_weight;
    double kappa_penalty_weight;
    double ref_s_weight;
    // std::string yamllocate="src/local/src/param.yaml";
public:
    PiecewiseJerkSpeedOptimizer();
    bool Process(double init_v, double init_a, Eigen::MatrixXd &localpath, std::vector<SpeedPoint> &speed_data,
                 double total_length, double total_time, std::vector<SpeedLimit> &speed_limit,
                 std::vector<SpeedDkappa> &curise_speed_, std::vector<SpeedPoint> &output_speed_data, YAML::Node &config);
    void loadyaml(YAML::Node &config);
    void binarysearch(const std::vector<SpeedPoint> &point, int &mindex, double s);
};