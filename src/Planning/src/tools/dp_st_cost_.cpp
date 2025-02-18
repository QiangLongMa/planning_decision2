#include"dp_st_cost_.h"
#include <complex>

dp_st_cost_::dp_st_cost_(/* args */){
    accel_cost_.fill(-1);
    jerk_cost_.fill(-1);
}

dp_st_cost_::~dp_st_cost_(){}

//计算 speed_cost
double dp_st_cost_::GetSpeedCost(const STPoint& first, const STPoint& second,
                              const double speed_limit,
                              const double cruise_speed) const{
    double cost = 0;
    const double speed = (second.s - first.s) / unit_t_;
    if (speed < 0){
        return kInf;//速度小于0时，返回较大的值 
    }
    double dex_speed = (speed-speed_limit)/speed_limit;//速度比率 
    if (dex_speed>0){ //speed > speed_limit
        cost+= exceed_speed_penalty*dex_speed + reference_speed_penalty*std::abs(speed-cruise_speed);//超速cost
        
    }else if(dex_speed<0){//speed < speed_limit
        cost+= -low_speed_penalty*dex_speed + reference_speed_penalty*std::abs(speed-cruise_speed);//低速cost
    }
    double diff_speed = speed - cruise_speed;
    cost += reference_speed_penalty*default_speed_cost*std::abs(diff_speed);
    return cost;
}

double dp_st_cost_::GetAccelCost(const double accel){
    double cost = 0.0;
    static constexpr double kEpsilon = 0.1;
    static constexpr size_t kShift = 100;
    //将accel的数值和在accel_cost_中的index联系起来
    //根据0<=10*accel+0.5+100<=200，得出程序默认accel正常在[-10,10]范围内，符合实际
    const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);
    if (accel_key >= accel_cost_.size()) {
        return kInf;
    }
    if (accel>max_acceleration||accel<max_deceleration){
        return kInf;
    }
    if (accel_cost_.at(accel_key)<0.0){
        const double accel_sq = accel * accel;
        if (accel > 0.0) {
        cost = accel_penalty * accel_sq;
        } else {
        cost = decel_penalty * accel_sq;
        }
        cost+= accel_sq * decel_penalty * decel_penalty /
                    (1 + std::exp(1.0 * (accel - max_deceleration))) +
                accel_sq * accel_penalty * accel_penalty /
                    (1 + std::exp(-1.0 * (accel - max_acceleration)));
        accel_cost_.at(accel_key) = cost;
    }else{
        //针对这个accel的cost早就被计算过，并保存在accel_cost_内
        cost =accel_cost_.at(accel_key); 
    }
    return cost*unit_t_;
}

// 计算 jerk_cost
double dp_st_cost_::JerkCost(const double jerk){
    double cost = 0.0;
    constexpr double kEpsilon = 0.1;
    constexpr size_t kShift = 200;
    //同上面accel cost的处理，认为jerk正常情况下在[-20,20]范围内
    const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);
    if (jerk_key>=jerk_cost_.size()){
        return kInf;
    }
    // if (jerk>max_jerk||jerk<min_jerk){
    //     return kInf;
    // }
    if (jerk_cost_.at(jerk_key) < 0.0){
        double jerk_sq = jerk * jerk;
        if (jerk>0){
            cost = positive_jerk_coeff*jerk_sq*unit_t_;
        }else{
            cost = negative_jerk_coeff*jerk_sq*unit_t_;
        }
        jerk_cost_.at(jerk_key) = cost;
    }else{
       jerk_cost_.at(jerk_key) = cost; 
    }
    return  cost;
} 

double dp_st_cost_::GetJerkCostByFourPoints(const STPoint& first,
                                         const STPoint& second,
                                         const STPoint& third,
                                         const STPoint& fourth) {
  double jerk = (fourth.s - 3 * third.s + 3 * second.s - first.s)/
                (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

double dp_st_cost_::GetAccelCostByTwoPoints(const double pre_speed,
                                         const STPoint& pre_point,
                                         const STPoint& curr_point){
    double current_speed = (curr_point.s - pre_point.s) / unit_t_;
    double accel = (current_speed - pre_speed) / unit_t_;
    return GetAccelCost(accel);                                        
}

double dp_st_cost_::GetAccelCostByThreePoints(const STPoint& first,
                                           const STPoint& second,
                                           const STPoint& third) {
  double accel = (first.s + third.s - 2 * second.s) / (unit_t_ * unit_t_);
  return GetAccelCost(accel);
}

double dp_st_cost_::GetJerkCostByTwoPoints(const double pre_speed,
                                        const double pre_acc,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point){
    const double curr_speed = (curr_point.s - pre_point.s) / unit_t_;
    const double curr_accel = (curr_speed - pre_speed) / unit_t_;
    const double jerk = (curr_accel - pre_acc) / unit_t_;
    return JerkCost(jerk);                                                                              
}

double dp_st_cost_::GetJerkCostByThreePoints(const double first_speed,
                                          const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) {
  const double pre_speed = (second.s - first.s) / unit_t_;
  const double pre_acc = (pre_speed - first_speed) / unit_t_;
  const double curr_speed = (third.s - second.s) / unit_t_;
  const double curr_acc = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

void dp_st_cost_::loadyaml(YAML::Node &config){
    //YAML::Node config = YAML::LoadFile(yamllocate);
    if(config["dp_st"]){
        YAML::Node lange_no_change = config["dp_st"];
        exceed_speed_penalty = lange_no_change["exceed_speed_penalty"].as<double>();
        default_speed_cost = lange_no_change["default_speed_cost"].as<double>();
        low_speed_penalty = lange_no_change["low_speed_penalty"].as<double>();
        reference_speed_penalty = lange_no_change["reference_speed_penalty"].as<double>();
        positive_jerk_coeff = lange_no_change["positive_jerk_coeff"].as<double>();
        negative_jerk_coeff = lange_no_change["negative_jerk_coeff"].as<double>();
        max_acceleration = lange_no_change["max_acceleration"].as<double>();
        max_deceleration = lange_no_change["max_deceleration"].as<double>();
        max_jerk = lange_no_change["max_jerk"].as<double>();
        min_jerk = lange_no_change["min_jerk"].as<double>();
        accel_penalty = lange_no_change["accel_penalty"].as<double>();
        decel_penalty = lange_no_change["decel_penalty"].as<double>();
        accel_weight = lange_no_change["accel_weight"].as<double>();
        jerk_weight = lange_no_change["jerk_weight"].as<double>();
    }
}


