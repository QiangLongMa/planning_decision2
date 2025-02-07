#include "scenario_manager.h"

ScenarioManager::ScenarioManager(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath, const Eigen::MatrixXd &obs_lidar): state_(ScenarioState::INIT), current_car_(car), current_globalPath_(globalPath), current_obs_lidar_(obs_lidar)
{
}

ScenarioState ScenarioManager::Update()
{
    // 根据当前状态判断下一步状态
    switch (state_)
    {
    case ScenarioState::INIT:
        // 判断是否需要转弯
        if (current_car_(0) > 10)
        {
            state_ = ScenarioState::TURN;
        }
        else
        {
            state_ = ScenarioState::STRAIGHT;
        }
        break;
    case ScenarioState::STRAIGHT:
        // 判断是否需要转弯
        if (current_car_(0) > 10)
        {
            state_ = ScenarioState::TURN;
        }
        break;
    case ScenarioState::TURN:
        // 判断是否需要直行
        if (current_car_(0) < 10)
        {
            state_ = ScenarioState::STRAIGHT;
        }
        break;
    case ScenarioState::NEAR_STOP:
        // 判断是否需要直行
        if (current_car_(0) < 10)
        {
            state_ = ScenarioState::STRAIGHT;
        }
        break;
    default:
        break;
    }

    return state_;
}