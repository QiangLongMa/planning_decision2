#include "scenario_tools.h"
namespace senarioTools
{
    /******************功能函数**************************/

    void findClosestPoint(const double &x, const double &y, const Eigen::MatrixXd &globalPath, int &carIndex)
    {
        int startIndex = std::max(carIndex - 10, 0);
        int endIndex = std::min(carIndex + 120, static_cast<int>(globalPath.cols()));
        double distance, d_min = std::numeric_limits<double>::max();
        for (int i = startIndex; i < endIndex; ++i)
        {
            distance = (std::pow(globalPath(0, i) - x, 2) + std::pow(globalPath(1, i) - y, 2));
            if (distance < d_min)
            {
                carIndex = i;
                d_min = distance;
            }
        }
    }

    void cartofrenet(const Eigen::VectorXd &CAR, const Eigen::MatrixXd &path, int &carIndex, frentPoint &carFrent)
    {
        const double dx = CAR(0) - path(0, carIndex);
        const double dy = CAR(1) - path(1, carIndex);
        const double theta = path(3, carIndex);
        const double cos_theta = std::cos(theta);
        const double sin_theta = std::sin(theta);
        double ref_s = path(6, carIndex);
        double path_s = dx * cos_theta + dy * sin_theta + ref_s;
        const double cross = cos_theta * dy - sin_theta * dx;
        carFrent.d = std::copysign(cross, cross);
        carFrent.s = std::abs(path_s);
    }
    // 获取与当前路径发生碰撞最近的s值
    void getclose_s(Eigen::MatrixXd &path, std::vector<Eigen::VectorXd> &obses,
                    double &min_s, bool &ischeck)
    {
        std::vector<Eigen::MatrixXd> all_local_points;
        Eigen::MatrixXd car_Fpoint;
        size_t path_size = path.cols();
        all_local_points.reserve(path_size);
        // 获取车辆的四个顶点的位置
        for (size_t i = 0; i < path_size; ++i)
        {
            obs::get_car_fourpoint(vehicle_length_, vehicle_width_, path.col(i), car_Fpoint); // 由车辆的中心点 计算局部路径上的每个店对应的车辆的四个顶点
            all_local_points.emplace_back(car_Fpoint);
        }
        std::vector<double> s_list;
        // 与所有的障碍物进行碰撞检测 获取 s_list
        for (auto &obs_ : obses)
        {
            size_t index;
            bool flag = obs::HasOverlap(all_local_points, obs_, path, index);
            if (flag)
            {                                                                 // 发生了碰撞
                double s = path(5, std::max(static_cast<int>(index - 1), 0)); // 碰撞发生时车辆所在位置的s
                s_list.push_back(s);
            }
        }
        if (s_list.size() != 0)
        {
            ischeck = true;
            std::sort(s_list.begin(), s_list.end());
            min_s = s_list.front();
        }
        else
        {
            ischeck = false;
        }
    }

    std::pair<bool, double> Pathplanningduringdeceleration(Eigen::MatrixXd &path, std::vector<Eigen::VectorXd> &GlobalcoordinatesystemObsesLimitinlocal_)
    {
        // 减速过程中依旧需要计算当前减速路径中有没有发生碰撞 以防止新的障碍物出现 而；车辆还在继续行使中  这是在计算之前的路
        double collision_detection_s;  // 当前路径发生碰撞时最近的s值
        bool collision_detection_flag; // 是否发生碰撞
        std::pair<bool, double> message;
        getclose_s(path, GlobalcoordinatesystemObsesLimitinlocal_, collision_detection_s, collision_detection_flag);
        message.first = collision_detection_flag;
        message.second = collision_detection_s;
        return message;
    }

    std::pair<double, double> ComputePositionProjection(
        const double x, const double y, const double ref_x, const double ref_y, const double theta, const double ref_s)
    {
        const double dx = x - ref_x;
        const double dy = y - ref_y;
        const double costheta = std::cos(theta);
        const double sintheta = std::sin(theta);
        std::pair<double, double> frenet_sd;
        frenet_sd.first = dx * costheta + dy * sintheta + ref_s;
        frenet_sd.second = dx * sintheta - dy * costheta;
        return frenet_sd;
    }

    // 不能使用轨迹拼接时的情况
    std::array<double, 6> ComputeReinitStitchingTrajectory(int localcloseindex,
                                                           int globalcloseindex, frentPoint &FrentPoint, Eigen::VectorXd &vehicle_state,
                                                           Eigen::MatrixXd &localpath, int indexinglobalpath, Eigen::MatrixXd &globalPath)
    {
        // 车辆启动
        // 利用运动学向外推0.1s的位置
        Eigen::Vector3d prev_vehicle_state;
        double prev_time = 0.1;
        double ref_steer = atan2(vehicle_state(4) * localpath(4, localcloseindex), 1); // 前轮转角
        double sinthea = std::sin(localpath(3, localcloseindex));
        double costhea = std::cos(localpath(3, localcloseindex));
        double param = std::tan(ref_steer) / vehicle_state(4);
        Eigen::Vector3d Parametervectors;
        Parametervectors << costhea, sinthea, param;
        Eigen::Vector3d curr_vehicle_state;
        curr_vehicle_state << vehicle_state(0), vehicle_state(1), vehicle_state(3);
        prev_vehicle_state = curr_vehicle_state + (Parametervectors * vehicle_state(2)) * prev_time; // 获得新的位置
        // prev_vehicle_state(2) = vehicle_state(3);
        // 求取新位置的 s d dl
        int carIndex = 0;
        int startIndex = std::max(indexinglobalpath - 20, 0);
        int endIndex = std::min(indexinglobalpath + 80, static_cast<int>(globalPath.cols()));
        double distance, d_min = std::numeric_limits<double>::max();
        double x = prev_vehicle_state(0);
        double y = prev_vehicle_state(1);
        double dx, dy;
        for (int i = startIndex; i < endIndex; ++i)
        {
            dx = globalPath(0, i) - x;
            dy = globalPath(1, i) - y;
            distance = dx * dx + dy * dy;
            if (distance < d_min)
            {
                carIndex = i;
                d_min = distance;
            }
        }
        frentPoint FrentPoint_;
        cartofrenet(prev_vehicle_state, globalPath, carIndex, FrentPoint_);
        double deltaYAW = tool::normalizeAngle(tool::d2r(prev_vehicle_state(2)) - globalPath(3, carIndex));
        double prev_dl = (1 - globalPath(4, carIndex) * FrentPoint_.d) * tan(deltaYAW);
        double ptr_kappa = globalPath(4, carIndex) / (1 - globalPath(4, carIndex) * FrentPoint_.d);
        double prev_ddl = localMath::Caldll(FrentPoint_.d, prev_dl, globalPath(5, carIndex), globalPath(4, carIndex), deltaYAW, ptr_kappa);
        std::array<double, 6> real_state = {FrentPoint_.s, FrentPoint_.d, prev_dl, prev_ddl,
                                            vehicle_state(2), vehicle_state(5)};
        return real_state;
    }

    std::array<double, 6> Decidestartsl(frentPoint &FrentPoint, int position_matched_index,
                                        int globalcloseindex, Eigen::MatrixXd &localpath,
                                        Eigen::MatrixXd &globalPath, Eigen::VectorXd &vehicle_state,
                                        std::vector<Eigen::Vector4d> &optTrajsd)
    {
        // double current_time = Time(); //获取当前的系统时间
        double planning_cycle_time = 0.1;            // 路径规划的时间
        int prev_trajectory_size = localpath.cols(); // 若是没有轨迹
        if (vehicle_state(2) < 0.27)
        { // 车辆速度较小默认为静止状态  起始状态 或者 人为刹车
            double deltaYAW = tool::normalizeAngle(tool::d2r(vehicle_state(3)) - globalPath(3, globalcloseindex));
            double vehicle_state_dl = (1 - globalPath(4, globalcloseindex) * FrentPoint.d) * tan(deltaYAW);
            double ptr_kappa = globalPath(4, globalcloseindex) / (1 - globalPath(4, globalcloseindex) * FrentPoint.d);
            double ptr_ddl = localMath::Caldll(FrentPoint.d, vehicle_state_dl, globalPath(5, globalcloseindex), globalPath(4, globalcloseindex), deltaYAW, ptr_kappa);
            std::array<double, 6> real_state = {FrentPoint.s, FrentPoint.d,
                                                vehicle_state_dl, ptr_ddl, vehicle_state(2), vehicle_state(5)};
            return real_state;
        }
        // 若是局部路径为空
        if (prev_trajectory_size == 0)
        {
            std::array<double, 6> vehicle_start_point =
                ComputeReinitStitchingTrajectory(position_matched_index, globalcloseindex, FrentPoint,
                                                 vehicle_state, localpath, globalcloseindex, globalPath);
            return vehicle_start_point;
        }
        auto frenet_sd =
            ComputePositionProjection(vehicle_state(0), vehicle_state(1),
                                      localpath(0, position_matched_index), localpath(1, position_matched_index),
                                      localpath(3, position_matched_index), localpath(5, position_matched_index)); // 计算横向误差
        auto lat_diff = frenet_sd.second;                                                                          // 横向误差
        // 此处是应对转弯时误差较大 车辆乱打方向
        constexpr double FLAGS_replan_lateral_distance_threshold = 0.5;
        if (std::abs(lat_diff) > FLAGS_replan_lateral_distance_threshold)
        {
            std::array<double, 6> vehicle_start_point =
                ComputeReinitStitchingTrajectory(position_matched_index, globalcloseindex, FrentPoint,
                                                 vehicle_state, localpath, globalcloseindex, globalPath);
            return vehicle_start_point;
        }
        auto prev_matched_index = std::min(position_matched_index, prev_trajectory_size - 1);
        int carIndex = 0;
        int startIndex = std::max(globalcloseindex - 80, 0);
        int endIndex = std::min(globalcloseindex + 80, static_cast<int>(globalPath.cols()));
        double distance, d_min = std::numeric_limits<double>::max();
        double x = localpath(0, prev_matched_index);
        double y = localpath(1, prev_matched_index);
        double dx;
        double dy;
        for (int i = startIndex; i < endIndex; ++i)
        {
            dx = globalPath(0, i) - x;
            dy = globalPath(1, i) - y;
            distance = dx * dx + dy * dy;
            if (distance < d_min)
            {
                carIndex = i;
                d_min = distance;
            }
        }
        frentPoint stitching_trajectoryFrentPoint;
        Eigen::Vector3d prev_state;
        prev_state << x, y, localpath(3, prev_matched_index);
        cartofrenet(prev_state, globalPath, carIndex, stitching_trajectoryFrentPoint);
        double path_s = stitching_trajectoryFrentPoint.s;
        // if (std::abs(stitching_trajectoryFrentPoint.s - FrentPoint.s) > 1.5) { //纵向距离差别太大了
        //     std::array<double, 6> vehicle_start_point =
        //           ComputeReinitStitchingTrajectory(position_matched_index, globalcloseindex,FrentPoint,
        //             vehicle_state,localpath );
        //     Trajectory_Splicing_Flag = false;
        //      return vehicle_start_point;
        // }
        double start_point_s;
        double start_point_l;
        double start_point_dl;
        double start_point_ddl;
        localMath::GetsldlBys(optTrajsd, path_s, start_point_s, start_point_l, start_point_dl, start_point_ddl);
        // start_point_l = stitching_trajectoryFrentPoint.d;
        // // std::cout<<"globalPath(3,carIndex): "<<globalPath(3,carIndex)<<std::endl;
        // // std::cout<<"prev_state(2): "<<prev_state(2)<<std::endl;
        // double deltaYAW = tool::normalizeAngle(prev_state(2) - globalPath(3, carIndex));
        // // std::cout<<"stitching_trajectoryFrentPoint.d: "<<stitching_trajectoryFrentPoint.d<<std::endl;
        // // std::cout<<"deltaYAW: "<<deltaYAW<<std::endl;
        // // std::cout<<"tan(deltaYAW: "<<tan(deltaYAW)<<std::endl;
        // double start_point_dl = (1 - globalPath(4, carIndex) * stitching_trajectoryFrentPoint.d) * tan(deltaYAW);
        // std::cout<<localpath.transpose()<<std::endl;
        std::array<double, 6> vehicle_start_point;
        vehicle_start_point = {start_point_s, start_point_l, start_point_dl, start_point_ddl,
                               localpath(2, prev_matched_index), localpath(6, prev_matched_index)};
        return vehicle_start_point;
    }

    // 检测当前车道内是否有障碍物
    bool CheckObstacleInLane(const std::vector<obses_sd> &obses_limit_SD)
    {
        // 定义左右车道的边界，根据实际定义调整
        const double rightMin = 0.0;
        const double rightMax = 3.0;
        // 车辆宽度
        const double car_wigth = 1.8;
        if (obses_limit_SD.size() == 0)
        {
            return false;
        }
        for (auto &obs : obses_limit_SD)
        {
        // 左侧空隙为obs.point1.l, obs.point2.l减去rightMin中较小的值
        // 右侧空隙为leftMax减去obs.point3.l, obs.point4.l中较小的值
            double leftSpace = std::min(obs.point1.l-rightMin, obs.point2.l - rightMin);
            double rightSpace = std::min(rightMax - obs.point3.l, rightMax - obs.point4.l);
            // 如果左侧空隙和右侧空隙都小于车辆宽度，则返回 true
            if (leftSpace < car_wigth && rightSpace < car_wigth)
            {
                return true;
            }
        }
        return false;
    }
}
