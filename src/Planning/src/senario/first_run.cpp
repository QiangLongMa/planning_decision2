#include "first_run.h"

// 通过初始化列表调用父类 Scenario 的构造函数
FirstRun::FirstRun(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath,
                   const std::vector<obses_sd> &obses_limit_SD,
                   const std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimit,
                   const double &gpsA)
    : Scenario(car, globalPath, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA)
{
}

void FirstRun::Straight() {
    heading_time_ = Time();
    // 1 为找到局部路径 0为未找到
    tool::frentPoint FrentPoint_;
    int first_run_index = 0;
    tool::cartesianToFrenet(car_, globalPath, FrentPoint_, first_run_index);
    double deltaYAW = tool::normalizeAngle(tool::d2r(car_(3)) - globalPath(3, first_run_index));
    double dl = (1 - globalPath(4, first_run_index) * FrentPoint_.d) * tan(deltaYAW);
    double ptr_kappa = globalPath(4, first_run_index) / (1 - globalPath(4, first_run_index) * FrentPoint_.d);
    ddl = localMath::Caldll(FrentPoint_.d, dl, globalPath(5, first_run_index), globalPath(4, first_run_index), deltaYAW, ptr_kappa);
    // 第一次运行时，只考虑直行  不考虑避障操作
    senarioTools::cartofrenet(car_, globalPath, index, FrentPoint_);
    // CalStartCarD(FrentPoint_.d, -1.5, start_l, end_l);
    delta_l = 0.5;
    target_v = 10;
    start_l = 1.5;
    end_l = -1.5;
    LOCAL_.setPatam(gpsA_, car_(2), FrentPoint_.s, FrentPoint_.d, dl, ddl, globalPath, 30, 10, index, obses_limit_SD, GlobalcoordinatesystemObsesLimit,
                    start_l, end_l, delta_l, target_v, -1.5, Decisionflags_, 0, true, false, 0, 0); // 最后一位时最近障碍物的位置
    find_local_path_ = LOCAL_.GetoptTrajxy(lastOptTrajxy, lastOptTrajsd);
    if (find_local_path_)
    { // 找到路径再进行之后的操作 确保初始时找到局部路径
        optTrajsd.clear();
        optTrajxy.resize(lastOptTrajxy.rows(), lastOptTrajxy.cols());
        optTrajxy = lastOptTrajxy;
        optTrajsd = lastOptTrajsd;
        lastOptTrajsd.clear();
    }
}

void FirstRun::MakeDecision()
{
    // 重置一下decision
    RestFlags();
    // 由于是fisrt run 场景比较简单，给直行的决策
    Decisionflags_.DriveStraightLineFlag = true;
}

bool FirstRun::Process()
{

    if (Decisionflags_.DriveStraightLineFlag)
        Straight();

    /****************************判断是否找到路径******************************************* */
    if (find_local_path_)
    {
        return true;
    }
    else
    {
        return false;
    }
}
