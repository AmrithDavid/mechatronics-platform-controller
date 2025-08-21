#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <thread>
#include <memory>
#include <atomic>
#include <chrono>
#include <vector>

// Note: This project was developed for ROS2/Gazebo simulation environment
// pfmsconnector handles communication with simulated platforms

class Controller: public ControllerInterface {
public:
    Controller();
    virtual ~Controller();

    // Pure virtual methods - must be implemented by derived classes
    virtual void run() = 0;
    virtual bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) = 0;
    virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, 
                                         pfms::geometry_msgs::Point goal, 
                                         double& distance, 
                                         double& time, 
                                         pfms::nav_msgs::Odometry& estimatedGoalPose) = 0;
    virtual double timeToGoal() = 0;
    virtual double distanceTravelled() = 0;
    
    // Implemented base functionality
    pfms::PlatformStatus status() override;
    pfms::PlatformType getPlatformType() override;
    double distanceToGoal() override;
    bool setTolerance(double tolerance) override;
    double timeTravelled() override;
    pfms::nav_msgs::Odometry getOdometry() override;
    std::vector<pfms::geometry_msgs::Point> getObstacles() override;
    
    void setType(pfms::PlatformType type);
    void setStatus(pfms::PlatformStatus newStatus);
    
    static constexpr double TARGET_SPEED = 1.0;  // Default speed for quadcopter

protected:
    std::vector<pfms::geometry_msgs::Point> goals_;
    std::shared_ptr<PfmsConnector> PfmsConnector_;  // ROS2 communication interface
    std::atomic<pfms::PlatformStatus> executionStatus_;  // Thread-safe status
    std::vector<std::pair<std::chrono::high_resolution_clock::time_point, 
                         std::chrono::high_resolution_clock::time_point>> motionIntervals;
    
    pfms::PlatformType platformType_;
    double tolerance_;
    double distance_;
    double time_;
    double currentPosX_, currentPosY_;
    
    void startMotion();
    void stopMotion();
};

#endif // CONTROLLER_H