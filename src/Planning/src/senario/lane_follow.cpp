#include "lane_follow.h"
// bool ObstacleAvoidanceFlag = false; // 街道超车的标志true为正在避障， false 直线行驶
// bool DecelerateFlag = false;        // 减速停车标志
// bool DriveStraightLineFlag = false; // 直线行使标志
// bool Overtakinginlaneflag = false;  // 车道内超车的标志
// bool righttoleftlane = false;       // 超车完成 返回原车道的标志
// 通过初始化列表调用父类 Scenario 的构造函数
LaneFollowScenario::LaneFollowScenario(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath,
                                       const std::vector<obses_sd> &obses_limit_SD,
                                       const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
                                       const double &gpsA, const double indexinglobalpath)  
    : Scenario(car, globalPath, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA, indexinglobalpath)
{





}

bool LaneFollowScenario::Straight() {
    start_l = -1.5; end_l = -1.5; target_l = -1.5; 
    delta_l = 0.5; target_v = 10;
    frentPoint FrentPoint_;
    int car_index;
    senarioTools::findClosestPoint(car_(0), car_(1), optTrajxy, car_index);
    senarioTools::cartofrenet(car_, optTrajxy, car_index, FrentPoint_);
     Eigen::VectorXd vehicle_state_(6);
    vehicle_state_ << car_(0), car_(1),car_(2),car_(3),car_(4), gpsA_;
    std::array<double, 6> vehicle_state = senarioTools::Decidestartsl(FrentPoint_, car_index, indexinglobalpath_,
                                        optTrajxy, globalPath, vehicle_state_, optTrajsd);
    local_start_s = vehicle_state[0];
    local_start_l = vehicle_state[1];
    dl = vehicle_state[2]; 
    ddl = vehicle_state[3];
    speed = vehicle_state[4];
    gpsA_ = vehicle_state[5];
    RestFlags(true, false, false, false, false);
    LOCAL_.setPatam(gpsA_, speed, FrentPoint_.s, FrentPoint_.d, dl, ddl, globalPath, 30, 10, car_index, obses_limit_SD, GlobalcoordinatesystemObsesLimit,
                    start_l, end_l, delta_l, target_v, target_l, Decisionflags_, 0, true, false, 0, 0); // 最后一位时最近障碍物的位置
    flag = LOCAL_.GetoptTrajxy(lastOptTrajxy, lastOptTrajsd);
    if (flag) {
        UpdateLocalPath();
    }
    return flag;
}

// 避障
bool LaneFollowScenario::AvoidObstacle() {
    start_l = -0.5; end_l = -2.5; target_l = -1.5; 
    delta_l = 0.5; target_v = 10;
    frentPoint FrentPoint_;
    int car_index;
    senarioTools::findClosestPoint(car_(0), car_(1), optTrajxy, car_index);
    senarioTools::cartofrenet(car_, optTrajxy, car_index, FrentPoint_);
     Eigen::VectorXd vehicle_state_(6);
    vehicle_state_ << car_(0), car_(1),car_(2),car_(3),car_(4), gpsA_;
    std::array<double, 6> vehicle_state = senarioTools::Decidestartsl(FrentPoint_, car_index, indexinglobalpath_,
                                        optTrajxy, globalPath, vehicle_state_, optTrajsd);
    local_start_s = vehicle_state[0];
    local_start_l = vehicle_state[1];
    dl = vehicle_state[2]; 
    ddl = vehicle_state[3];
    speed = vehicle_state[4];
    gpsA_ = vehicle_state[5];
    RestFlags(false, false, false, true, false);
    LOCAL_.setPatam(gpsA_, speed, FrentPoint_.s, FrentPoint_.d, dl, ddl, globalPath, 30, 10, car_index, obses_limit_SD, GlobalcoordinatesystemObsesLimit,
                    start_l, end_l, delta_l, target_v, target_l, Decisionflags_, 0, true, false, 0, 0);
    flag = LOCAL_.GetoptTrajxy(lastOptTrajxy, lastOptTrajsd);
    if (flag) {
        UpdateLocalPath();
    }
    return flag;
}
void LaneFollowScenario::Overtake()
{
}

//减速停车 目前的逻辑是先获取直线行使的局部路径 然后使用获取的路径与当前的障碍物进行碰撞，
//获取与自车发生碰撞的s， 之后减速停车 如果s较小 则触发AEB
bool LaneFollowScenario::DecelerateFollow() {
    //创建两个空的局部路径生成的障碍物信息，目的为了通过里面的障碍物检测模块
    std::vector<obses_sd> temporary_obses_limit_SD;
    std::vector<Eigen::VectorXd> temporary_GlobalcoordinatesystemObsesLimit;
    start_l = -0.5; end_l = -2.5; target_l = -1.5; 
    delta_l = 0.5; target_v = 10;
    frentPoint FrentPoint_;
    int car_index;
    senarioTools::findClosestPoint(car_(0), car_(1), optTrajxy, car_index);
    senarioTools::cartofrenet(car_, optTrajxy, car_index, FrentPoint_);
     Eigen::VectorXd vehicle_state_(6);
    vehicle_state_ << car_(0), car_(1),car_(2),car_(3),car_(4), gpsA_;
    std::array<double, 6> vehicle_state = senarioTools::Decidestartsl(FrentPoint_, car_index, indexinglobalpath_,
                                        optTrajxy, globalPath, vehicle_state_, optTrajsd);
    local_start_s = vehicle_state[0];
    local_start_l = vehicle_state[1];
    dl = vehicle_state[2]; 
    ddl = vehicle_state[3];
    speed = vehicle_state[4];
    gpsA_ = vehicle_state[5];
    RestFlags(true, false, false, false, false);
    LOCAL_.setPatam(gpsA_, speed, FrentPoint_.s, FrentPoint_.d, dl, ddl, globalPath, 30, 10, car_index, obses_limit_SD, GlobalcoordinatesystemObsesLimit,
                    start_l, end_l, delta_l, target_v, target_l, Decisionflags_, 0, true, false, 0, 0);
    if (!LOCAL_.GetoptTrajxy(lastOptTrajxy, lastOptTrajsd)) {
        return false;
    } 
    //若获取当前的行使路径
    std::pair<bool, double> CollisionAndS;
    CollisionAndS = senarioTools::Pathplanningduringdeceleration(lastOptTrajxy, GlobalcoordinatesystemObsesLimit);
    if (!CollisionAndS.first) { //如果当前路径没有碰撞 
        UpdateLocalPath();
        return true; 
    }
    if (CollisionAndS.second > distance_threshold) {
        start_l = -1.0; end_l = -2; target_l = -1.5; 
        delta_l = 0.5; target_v = 0;
        int LengthLocalPath = CollisionAndS.second - distance_threshold ; //局部路径的长度 
        RestFlags(false, false, true, false, false);//设置标志符
        LOCAL_.setPatam(gpsA_, speed, FrentPoint_.s, FrentPoint_.d, dl, ddl, globalPath, LengthLocalPath, LengthLocalPath, car_index, obses_limit_SD, GlobalcoordinatesystemObsesLimit,
                    start_l, end_l, delta_l, target_v, target_l, Decisionflags_, 0, false, false, 0, 0);
        flag = LOCAL_.GetoptTrajxy(lastOptTrajxy, lastOptTrajsd);
        if (flag) {
            UpdateLocalPath();
        }
        return flag;
    } else {
        /**********AEB************/
    }
}

void LaneFollowScenario::ReturnRightLane()
{
}

void LaneFollowScenario::MakeDecision()
{
    // 判断迭代容器vector obses_limit_SD 是否为空
  
}



bool LaneFollowScenario::Process()
{
    return false;
}
