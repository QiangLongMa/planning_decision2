#include "scenario.h"

constexpr int64_t NANOS_PER_SECOND = 1000000000; // 1e9

// 使用初始化列表来初始化成员变量
Scenario::Scenario(const Eigen::VectorXd &car,
                   const Eigen::MatrixXd &globalPath,
                   const std::vector<obses_sd> &obses_limit_SD,
                   const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
                   const double &gpsA) : car_(car),
                   globalPath(globalPath),
                   obses_limit_SD(obses_limit_SD),
                   GlobalcoordinatesystemObsesLimit(GlobalcoordinatesystemObsesLimit),
                   gpsA_(gpsA)
{
}



double Scenario::Time()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto nano_time_point = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    auto epoch = nano_time_point.time_since_epoch();
    uint64_t now_nano = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
    return static_cast<double>(now_nano) / NANOS_PER_SECOND;
}

// bool ObstacleAvoidanceFlag = false; // 街道超车的标志true为正在避障， false 直线行驶
// bool DecelerateFlag = false;        // 减速停车标志
// bool DriveStraightLineFlag = false; // 直线行使标志
// bool Overtakinginlaneflag = false;  // 车道内超车的标志
// bool righttoleftlane = false;       // 超车完成 返回原车道的标志
void Scenario::RestFlags()
{
    Decisionflags_.DriveStraightLineFlag = false; // 直行决策
    Decisionflags_.ObstacleAvoidanceFlag = false; // 避障决策
    Decisionflags_.DecelerateFlag = false;        // 减速跟车决策
    Decisionflags_.Overtakinginlaneflag = false;  // 超车决策
    Decisionflags_.righttoleftlane = false;       // 返回原车道决策
}
