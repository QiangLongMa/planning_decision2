#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <vector>
// #include"local_dp_qp.h"
#include "QP.h"
#include "messages.hpp"


namespace obs{

    void obsfindClosestPoint(double x, double y, Eigen::MatrixXd &globalPath, int &minIndex, const int carIndex);
    void findClosestPoint(const Eigen::VectorXd &car, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &path, int &minIndex);
    int cartesianToFrenet(const Eigen::VectorXd &car, const Eigen::Matrix<double, 7, Eigen::Dynamic> &path, frentPoint &carFrentPoint, int &index);
    // 计算障碍物的四个顶点和中心点的sl
    void lidar_obs_Frenet(const Eigen::VectorXd &realPosition, Eigen::MatrixXd &globalPath, obses_sd &obsfrent, int carindex);

    void CalculateobsesSD(frentPoint vehicle_position, Eigen::MatrixXd &globalPath_, Eigen::MatrixXd &obs, std::vector<obses_sd> &sd, std::vector<Eigen::VectorXd> &obses_limit,
                          Eigen::MatrixXd &obses_base_lidar, std::vector<Eigen::VectorXd> &obs_limits_distance, int carindex);

    void get_car_fourpoint(double length, double wigth, Eigen::VectorXd car_center, Eigen::MatrixXd &car_Fpoint);

    // 面积比较的方法进行碰撞检测  carpoint 为局部路径上的点 obses为障碍物上的坐标
    int Area_Comparison(std::vector<Eigen::MatrixXd> &carpoint, std::vector<Eigen::VectorXd> &obses);

    std::array<double, 2> FindSL(Eigen::MatrixXd &globalPath, int minIndex, int carindex, double x, double y);

    int decide_obs_true_false(obses_sd &sd, double left_bound, double right_bound);

    bool HasOverlap(std::vector<Eigen::MatrixXd> &carpoint_points, std::vector<Eigen::VectorXd> &obses, Eigen::MatrixXd &opty);
    bool HasOverlap(Eigen::MatrixXd &car_point, std::vector<Eigen::VectorXd> &obses, double thea);

    bool HasOverlap(std::vector<Eigen::MatrixXd> &carpoint_points, Eigen::VectorXd &obs, Eigen::MatrixXd &opty, size_t &index);
}
