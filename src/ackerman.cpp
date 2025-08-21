#include "ackerman.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <thread>

Ackerman::Ackerman() : Controller(), i(0) {
    setTolerance(0.5);
    setType(pfms::PlatformType::ACKERMAN);
}

Ackerman::~Ackerman() {
    PfmsConnector_.reset();
}

void Ackerman::run() {
    for (auto goal : goals_) {
        pfms::nav_msgs::Odometry currentOdo = getOdometry();
        startMotion();
        setStatus(pfms::PlatformStatus::RUNNING);
        
        double distance = distanceToGoal();
        
        // Acceleration phase - while more than 1.5m from goal
        while (distance > 1.5) {
            // Simple proportional steering towards goal
            double dx = goal.x - currentOdo.position.x;
            double dy = goal.y - currentOdo.position.y;
            double angleToGoal = atan2(dy, dx);
            double angleDiff = angleToGoal - currentOdo.yaw;
            
            // Normalize angle difference to [-pi, pi]
            while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
            while (angleDiff < -M_PI) angleDiff += 2 * M_PI;
            
            steering = angleDiff * 0.5;  // Proportional steering
            steering = std::max(-0.5, std::min(0.5, steering));  // Limit steering
            
            throttle = 0.2;
            brake = 0.0;
            
            pfms::commands::Ackerman cmd{i++, brake, steering, throttle};
            PfmsConnector_->send(cmd);
            
            currentOdo = getOdometry();
            distance = distanceToGoal();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Deceleration phase - within 1.5m of goal
        while (distance > tolerance_) {
            double dx = goal.x - currentOdo.position.x;
            double dy = goal.y - currentOdo.position.y;
            double angleToGoal = atan2(dy, dx);
            double angleDiff = angleToGoal - currentOdo.yaw;
            
            while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
            while (angleDiff < -M_PI) angleDiff += 2 * M_PI;
            
            steering = angleDiff * 0.5;
            steering = std::max(-0.5, std::min(0.5, steering));
            
            throttle = 0.0;
            brake = 300.0;
            
            pfms::commands::Ackerman cmd{i++, brake, steering, throttle};
            PfmsConnector_->send(cmd);
            
            currentOdo = getOdometry();
            distance = distanceToGoal();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Full stop at goal
        throttle = 0.0;
        brake = 8000.0;
        steering = 0.0;
        
        pfms::commands::Ackerman cmd{i++, brake, steering, throttle};
        PfmsConnector_->send(cmd);
        
        stopMotion();
    }
    
    setStatus(pfms::PlatformStatus::IDLE);
}

bool Ackerman::setGoals(std::vector<pfms::geometry_msgs::Point> goals) {
    pfms::nav_msgs::Odometry odo = getOdometry();
    pfms::nav_msgs::Odometry estimatedGoalPose;
    goals_ = goals;
    
    bool allReachable = true;
    for (auto &goal : goals) {
        if (!checkOriginToDestination(odo, goal, distance_, time_, estimatedGoalPose)) {
            allReachable = false;
            break;
        }
    }
    return allReachable;
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double &distance,
                                        double &time,
                                        pfms::nav_msgs::Odometry &estimatedGoalPose) {
    double dx = goal.x - origin.position.x;
    double dy = goal.y - origin.position.y;
    distance = sqrt(dx*dx + dy*dy);
    
    double maxSpeed = 2.91;
    time = distance / maxSpeed;
    
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.yaw = atan2(dy, dx);
    
    return true;  // Simplified - always reachable
}

double Ackerman::timeToGoal() {
    double maxSpeed = 2.91;
    time_ = distance_ / maxSpeed;
    return time_;
}

double Ackerman::distanceTravelled() {
    double totalDistance = 0.0;
    pfms::nav_msgs::Odometry lastOdometry = getOdometry();
    
    for (const auto& interval : motionIntervals) {
        pfms::nav_msgs::Odometry currentOdometry = getOdometry();
        double dx = currentOdometry.position.x - lastOdometry.position.x;
        double dy = currentOdometry.position.y - lastOdometry.position.y;
        double intervalDistance = std::sqrt(dx * dx + dy * dy);
        lastOdometry = currentOdometry;
        totalDistance += intervalDistance;
    }
    
    return totalDistance;
}