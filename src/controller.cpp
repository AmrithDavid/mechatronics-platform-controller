#include "controller.h"

Controller::Controller()
    : PfmsConnector_(std::make_shared<PfmsConnector>()), executionStatus_(pfms::PlatformStatus::IDLE) {
}

Controller::~Controller() {
}

void Controller::setStatus(pfms::PlatformStatus newStatus) {
    executionStatus_.store(newStatus);
    PfmsConnector_->send(newStatus);
}

pfms::PlatformStatus Controller::status() {
    return executionStatus_.load();
}

double Controller::timeTravelled() {
    using namespace std::chrono;
    double totalTime = 0.0;
    for (const auto& interval : motionIntervals) {
        auto intervalDuration = interval.second - interval.first;
        auto durationInSeconds = duration_cast<duration<double>>(intervalDuration);
        totalTime += durationInSeconds.count();
    }
    return totalTime;
}

void Controller::startMotion() {
    motionIntervals.push_back({std::chrono::high_resolution_clock::now(), {}});
}

void Controller::stopMotion() {
    if (!motionIntervals.empty() && motionIntervals.back().second == std::chrono::high_resolution_clock::time_point{}) {
        motionIntervals.back().second = std::chrono::high_resolution_clock::now();
    }
}

void Controller::setType(pfms::PlatformType type) {
    platformType_ = type;
}

pfms::PlatformType Controller::getPlatformType() {
    return platformType_;
}

bool Controller::setTolerance(double newTolerance) {
    if (newTolerance >= 0) {
        tolerance_ = newTolerance;
        return true;
    }
    return false;
}

pfms::nav_msgs::Odometry Controller::getOdometry() {
    pfms::nav_msgs::Odometry odo;
    if (PfmsConnector_) {
        PfmsConnector_->read(odo, getPlatformType());
    }
    return odo;
}

double Controller::distanceToGoal() {
    pfms::nav_msgs::Odometry odo = getOdometry();
    currentPosX_ = odo.position.x;
    currentPosY_ = odo.position.y;

    if (goals_.empty()) {
        return 0.0;
    }

    double currentGoalx = goals_.front().x;
    double currentGoaly = goals_.front().y;
    double squaredDistance = pow((currentGoaly - currentPosY_), 2) + pow((currentGoalx - currentPosX_), 2);
    distance_ = sqrt(squaredDistance);

    pfms::nav_msgs::Odometry estimatedGoalPose;
    checkOriginToDestination(odo, goals_.front(), distance_, time_, estimatedGoalPose);

    return distance_;
}

std::vector<pfms::geometry_msgs::Point> Controller::getObstacles() {
    std::vector<pfms::geometry_msgs::Point> obstacles;
    return obstacles;
}