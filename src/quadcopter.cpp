#include "quadcopter.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

Quadcopter::Quadcopter() : Controller(), ii(0) {
    setTolerance(0.5);
    setType(pfms::PlatformType::QUADCOPTER);
}

Quadcopter::~Quadcopter() {
    PfmsConnector_.reset();
}

bool Quadcopter::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose) {
    double dx = goal.x - origin.position.x;
    double dy = goal.y - origin.position.y;
    
    distance = std::hypot(dx, dy);
    time = distance / TARGET_SPEED;
    
    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.yaw = origin.yaw;
    estimatedGoalPose.linear.x = 0;
    estimatedGoalPose.linear.y = 0;
    
    return true;
}

void Quadcopter::run() {
    setStatus(pfms::PlatformStatus::TAKEOFF);
    setStatus(pfms::PlatformStatus::RUNNING);
    
    for (auto& goal : goals_) {
        pfms::nav_msgs::Odometry currentOdo = getOdometry();
        startMotion();
        
        while (distanceToGoal() > tolerance_) {
            double dx = goal.x - currentOdo.position.x;
            double dy = goal.y - currentOdo.position.y;
            double dz = 2.0 - currentOdo.position.z;  // Target altitude 2m
            double target_angle = std::atan2(dy, dx);
            
            // Normalize angle
            if (target_angle > M_PI) {
                target_angle -= 2 * M_PI;
            } else if (target_angle < -M_PI) {
                target_angle += 2 * M_PI;
            }
            
            // Adaptive speed based on distance
            double distance = std::hypot(dx, dy);
            double speed = std::min(TARGET_SPEED, distance / 2.0);
            
            double move_f_b = speed * std::cos(target_angle);
            double move_l_r = speed * std::sin(target_angle);
            double move_u_d = dz;
            
            // Altitude adjustments
            if (currentOdo.position.z > 2.1) {
                move_u_d = -0.1;
            } else if (currentOdo.position.z < 1.9) {
                move_u_d = 0.1;
            }
            
            pfms::commands::Quadcopter cmd = {ii++, 0, move_l_r, move_u_d, move_f_b};
            PfmsConnector_->send(cmd);
            
            currentOdo = getOdometry();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        stopMotion();
    }
    
    setStatus(pfms::PlatformStatus::IDLE);
}

bool Quadcopter::setGoals(std::vector<pfms::geometry_msgs::Point> goals) {
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

double Quadcopter::timeToGoal() {
    double distance = distanceToGoal();
    return distance / TARGET_SPEED;
}

double Quadcopter::distanceTravelled() {
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