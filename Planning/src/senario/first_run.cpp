#include "first_run.h"

FirstRun::FirstRun()
{
    Decisionflags_.DriveStraightLineFlag = true;
}

void FirstRun::Straight()
{
    heading_time_ = Time();
    local_dp_qp LOCAL_;
    double gpsA = gpsA_;
    // 1 为找到局部路径 0为未找到
    tool::frentPoint FrentPoint_;
    int first_run_index = 0;
    tool::cartesianToFrenet(car, globalPath, FrentPoint_, first_run_index);
    index = first_run_index; /// car index
    double deltaYAW = tool::normalizeAngle(tool::d2r(car(3)) - globalPath(3, first_run_index));
    double dl = (1 - globalPath(4, first_run_index) * FrentPoint_.d) * tan(deltaYAW);
    double ptr_kappa = globalPath(4, first_run_index) / (1 - globalPath(4, first_run_index) * FrentPoint_.d);
    ddl = Caldll(FrentPoint_.d, dl, globalPath(5, first_run_index), globalPath(4, first_run_index), deltaYAW, ptr_kappa);
    // 第一次运行时，只考虑直行  不考虑避障操作
    cartofrenet(car, globalPath, index, FrentPoint_);
    // CalStartCarD(FrentPoint_.d, -1.5, start_l, end_l);
    delta_l = 0.5;
    target_v = 10;
    start_l = 1.5;
    end_l = -1.5;
    LOCAL_.setPatam(gpsA, car(2), FrentPoint_.s, FrentPoint_.d, dl, ddl, globalPath, 30, 10, index, obses_limit_SD, GlobalcoordinatesystemObsesLimit,
                    start_l, end_l, delta_l, target_v, -1.5, Decisionflags_, 0, true, false, 0, 0); // 最后一位时最近障碍物的位置
    none = LOCAL_.GetoptTrajxy(lastOptTrajxy, lastOptTrajsd);
    if (none)
    { // 找到路径再进行之后的操作 确保初始时找到局部路径
    }
}