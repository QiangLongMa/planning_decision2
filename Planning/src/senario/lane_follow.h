#include "scenario.h"

class LaneFollowScenario : public Scenario
{

    bool IsTransferable(const Scenario &other_scenario, const Eigen::MatrixXd &obs_cam, const Eigen::MatrixXd &obs_lidar, const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath) override ;


}