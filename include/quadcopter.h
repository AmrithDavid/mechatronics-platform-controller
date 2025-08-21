#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"
#include <vector>
#include <atomic>
#include <chrono>

class Quadcopter : public Controller {
public:
    Quadcopter();
    virtual ~Quadcopter();

    // Override virtual methods from Controller
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                 pfms::geometry_msgs::Point goal,
                                 double& distance,
                                 double& time,
                                 pfms::nav_msgs::Odometry& estimatedGoalPose) override;
    void run() override;
    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;
    double timeToGoal() override;
    double distanceTravelled() override;

private:
    unsigned long ii;  // Command sequence number
    double currentPositionX_, currentPositionY_;
};

#endif // QUADCOPTER_H