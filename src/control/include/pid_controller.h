#include <fstream>
#include<iostream>
#include <Eigen/Eigen>
#include<Eigen/Dense>


class pidcontroller
{
private:
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double kaw_ = 0.0;
    double previous_error_ = 0.0;
    double previous_output_ = 0.0;
    double integral_ = 0.0;
    double integrator_saturation_high_ = 0.0;
    double integrator_saturation_low_ = 0.0;
    bool first_hit_ = false;
    bool integrator_enabled_ = false;
    bool integrator_hold_ = false;
    int integrator_saturation_status_ = 0;
    // Only used for pid_BC_controller and pid_IC_controller
    double output_saturation_high_ = 0.0;
    double output_saturation_low_ = 0.0;
    int output_saturation_status_ = 0;
    double ts = 0.01;
    
public:
    pidcontroller(/* args */);
    ~pidcontroller();
    void Init(double kp , double ki , double kd, bool integrator_enabled, double  integrator_saturation_leve);
    double Control(double error, double dt);
    void Reset();
};

