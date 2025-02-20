#include "pid_controller.h"



pidcontroller::pidcontroller(/* args */)
{
}

pidcontroller::~pidcontroller()
{
}
double pidcontroller::Control(double error, double dt){
    if(dt <= 0) {
        return previous_output_;
    }
    double diff = 0;
    double output = 0;
    if(first_hit_) {
        first_hit_ = false;
    } else {
        diff = (error - previous_error_) / dt;
    }
    if(!integrator_enabled_) {//如果积分器被禁用为假
        integral_ =0;//重置积分项为0
    } else if (!integrator_hold_) {//如果积分器未保持为假
        // 对积分值 integral_ 应用饱和值限制，避免积分项超出预设的最大和最小值。如果超出范围，将其限制到饱和值，并设置饱和状态标记 integrator_saturation_status_。
        integral_ += error * dt * ki_;
        if(integral_ > integrator_saturation_high_){
            integrator_saturation_status_ = 1;
            integral_ = integrator_saturation_high_;
        } else if (integral_ < integrator_saturation_low_){
            integral_ = integrator_saturation_low_;
            integrator_saturation_status_ = -1;
        } else {
            integrator_saturation_status_ = 0;
        }
    }

    previous_error_ = error;
    output = error * kp_ + integral_ + diff * kd_;
    previous_output_ = output;
    return output;
}

void pidcontroller::Init(double kp , double ki , double kd,
                        bool integrator_enabled, double  integrator_saturation_leve){
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
    integrator_enabled_ = integrator_enabled;
    integrator_saturation_high_ = std::fabs(integrator_saturation_leve);
    integrator_saturation_low_ = -std::fabs(integrator_saturation_leve);
    integrator_saturation_status_ = 0;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    integrator_hold_ = false;
}

void pidcontroller::Reset(){
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
    integrator_saturation_status_ = 0;
}