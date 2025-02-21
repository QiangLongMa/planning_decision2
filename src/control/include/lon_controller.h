#include <fstream>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include "pid_controller.h"
#include <chrono>
#include <limits>
#define M_PI       3.14159265358979323846   // pi
#define NANOS_PER_SECOND  1e9

class loncontroller
{
private:
    pidcontroller station_pid_controller_;
    pidcontroller speed_pid_controller_;
    double ts = 0.01;
    size_t closeindex_;
    std::tuple<double, double,double, double, double> vehicle_state_;
    bool FLAGS_enable_speed_station_preview = true;
    double preview_station_error = 0.0;
    double preview_speed_error = 0.0;
    double preview_acceleration_reference = 0.0;
    double station_error = 0.0;
    double speed_error = 0.0;
    double header_time_;
public:
    loncontroller(/* args */);
    ~loncontroller();
    void Init(const size_t closeindex, std::tuple<double, double,double, double, double> vehicle_state, double heading_time);
    bool ComputeControlCommand(const Eigen::MatrixXd &localpath,  double &acceleration_cmd);
    void ComputeLongitudinalErrors(const Eigen::MatrixXd &localpath,
                    const double preview_time, const double ts);
    void ToTrajectoryFrame(const double x, const double y,
                                      const double theta, const double v,
                                    std::array<double ,5> &ref_point,
                                    double *ptr_s, double *ptr_s_dot,double *ptr_d,
                                    double *ptr_d_dot);
    double NormalizeAngle(const double angle);
    double Clamp(const double value, double bound1, double bound2);
    void Reset();
    double Time();
    void QueryNearestPointByAbsoluteTime(double t, const Eigen::MatrixXd &localpath, std::array<double, 7>&point);
};

