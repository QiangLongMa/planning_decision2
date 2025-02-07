#include "scenario.h"

class FirstRun : public Scenario
{
public:
    FirstRun(const Eigen::VectorXd &car, const Eigen::MatrixXd &globalPath, const Eigen::MatrixXd &obs_lidar);
    void Straight() override;
    void AvoidObstacle() override ;
    void Overtake() override ;
    void DecelerateFollow() override ;
    void EmergencyStop() override ;
}
