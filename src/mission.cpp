#include "mission.h"
#include <thread>
#include <mutex>

Mission::Mission(std::vector<ControllerInterface*> controllers)
    : controllers_(controllers) {}

Mission::~Mission() {}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform) {
    std::lock_guard<std::mutex> lock(mutex_);
    platformGoals_[platform] = goals;
}

bool Mission::run() {
    std::vector<std::thread> threads;
    
    for (auto& controller : controllers_) {
        if (controller->getPlatformType() == pfms::QUADCOPTER) {
            threads.emplace_back([&] {
                controller->setGoals(platformGoals_[pfms::QUADCOPTER]);
                controller->run();
            });
        } else if (controller->getPlatformType() == pfms::ACKERMAN) {
            threads.emplace_back([&] {
                controller->setGoals(platformGoals_[pfms::ACKERMAN]);
                controller->run();
            });
        }
    }
    
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    return missionPossible_.load();
}

std::vector<unsigned int> Mission::status() {
    std::vector<unsigned int> completionStatus(controllers_.size());
    double totalDistance = 0;
    
    for (auto& goal : platformGoals_) {
        for (auto& point : goal.second) {
            totalDistance += sqrt(point.x * point.x + point.y * point.y);
        }
    }
    
    double distanceTravelled = 0;
    for (size_t i = 0; i < controllers_.size(); ++i) {
        distanceTravelled += controllers_[i]->distanceTravelled();
        completionStatus[i] = static_cast<unsigned int>((distanceTravelled / totalDistance) * 100);
    }
    
    return completionStatus;
}

void Mission::setMissionObjective(mission::Objective objective) {
    currentObjective_ = objective;
}

std::vector<double> Mission::getDistanceTravelled() {
    std::vector<double> distances(controllers_.size());
    for (size_t i = 0; i < controllers_.size(); ++i) {
        distances[i] = controllers_[i]->distanceTravelled();
    }
    return distances;
}

std::vector<double> Mission::getTimeMoving() {
    std::vector<double> times(controllers_.size());
    for (size_t i = 0; i < controllers_.size(); ++i) {
        times[i] = controllers_[i]->timeTravelled();
    }
    return times;
}

std::vector<std::pair<int, int>> Mission::getPlatformGoalAssociation() {
    std::vector<std::pair<int, int>> associations;
    for (auto& pair : platformGoals_) {
        for (size_t i = 0; i < pair.second.size(); i++) {
            associations.emplace_back(static_cast<int>(pair.first), i);
        }
    }
    return associations;
}