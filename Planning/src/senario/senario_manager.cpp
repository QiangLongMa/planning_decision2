#include "scenario_manager.h"
#include <algorithm>

ScenarioManager::ScenarioManager(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath, const Eigen::MatrixXd &obs_lidar) : state_(ScenarioState::INIT), current_car_(car), current_globalPath_(globalPath), current_obs_lidar_(obs_lidar)
{
}

ScenarioState ScenarioManager::Update()
{
    // 根据当前状态判断下一步状态
    switch (state_)
    {
    case ScenarioState::INIT:
        // 判断是否需要转弯
        if (first_run_)
        {
            state_ = ScenarioState::INIT;
        }
        else
        {
            state_ = ScenarioState::STRAIGHT;
        }
        break;
    case ScenarioState::STRAIGHT:
        // 判断是否是路口
        if (IsApproachingIntersection())
        {
            state_ = ScenarioState::TURN;
        }
        else if (IsApproachingStopLine())
        {
            state_ = ScenarioState::NEAR_STOP;
        }
        break;
    case ScenarioState::TURN:
        // 判断是否需要直行
        if (IsStraightForward())
        {
            state_ = ScenarioState::STRAIGHT;
        }
        else if (IsApproachingStopLine())
        {
            state_ = ScenarioState::NEAR_STOP;
        }
        break;
    case ScenarioState::NEAR_STOP:
        // 判断是否需要直行
        state_ = ScenarioState::STRAIGHT;
        break;
    default:
        break;
    }

    return state_;
}

void ScenarioManager::UpdateData(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath, const Eigen::MatrixXd &obs_lidar)
{
    // 只更新实时数据，保持内部的状态 state_ 不变
    current_car_ = car;
    current_globalPath_ = globalPath;
    current_obs_lidar_ = obs_lidar;
}

bool ScenarioManager::IsApproachingIntersection()
{
    findClosestPoint(current_car_(0), current_car_(1), current_globalPath_, index); // 寻找此时车辆在全局路径的索引
    cartofrenet(current_car_, current_globalPath_, index, FrentPoint_);
    size_t end_index = std::min(index + 120, static_cast<int>(current_globalPath_.cols())); // 30m的距离
    size_t start_index_ = std::max(static_cast<size_t>(0.0), start_index);                  // 5m
    for (size_t i = start_index_; i < end_index; ++i)
    {
        if (std::abs(current_globalPath_(4, i)) >= 0.03)
        {
            return true;
        }
    }
    return false;
    // 判断是否接近路口
}

bool ScenarioManager::IsApproachingStopLine()
{
    findClosestPoint(current_car_(0), current_car_(1), current_globalPath_, index); // 寻找此时车辆在全局路径的索引

    // 车辆距离终点距离大于30
    if (current_globalPath_(6, current_globalPath_.cols() - 1) - current_globalPath_(6, mindex) >= 30)
    {
        return false;
    }
    // 车辆距离终点距离小于 30
    else
    {
        return true;
    }
}

bool ScenarioManager::IsStraightForward() // 判断是否直行状态，车辆30米范围内没有路口，或者曲率特别大的地方
{
    findClosestPoint(current_car_(0), current_car_(1), current_globalPath_, index); // 寻找此时车辆在全局路径的索引
    cartofrenet(current_car_, current_globalPath_, index, FrentPoint_);
    size_t end_index = std::min(index + 120, static_cast<int>(current_globalPath_.cols())); // 30m的距离
    size_t start_index_ = std::max(static_cast<size_t>(0.0), start_index);                  // 5m
    for (size_t i = start_index_; i < end_index; ++i)
    {
        if (std::abs(current_globalPath_(4, i)) >= 0.03)
        {
            return false;
        }
    }
    return true;
}

bool ScenarioManager::ChangeFirstRun()
{
    first_run_ = false;
    return first_run_;
}

// 重新设置状态
void ScenarioManager::Reset()
{
    state_ = ScenarioState::INIT;
    first_run_ = true;
}
