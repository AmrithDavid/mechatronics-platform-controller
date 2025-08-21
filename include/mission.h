#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include <map>
#include <mutex>
#include <atomic>
#include "missioninterface.h"
#include "controllerinterface.h"

class Mission : public MissionInterface {
public:
    Mission(std::vector<ControllerInterface*> controllers);
    virtual ~Mission();

    // Mission control methods
    void setGoals(std::vector<pfms::geometry_msgs::Point> goals, 
                 pfms::PlatformType platform) override;
    bool run() override;
    std::vector<unsigned int> status() override;
    void setMissionObjective(mission::Objective objective) override;
    
    // Monitoring methods
    std::vector<double> getDistanceTravelled() override;
    std::vector<double> getTimeMoving() override;
    std::vector<std::pair<int, int>> getPlatformGoalAssociation() override;

private:
    std::vector<ControllerInterface*> controllers_;
    std::map<pfms::PlatformType, std::vector<pfms::geometry_msgs::Point>> platformGoals_;
    mission::Objective currentObjective_{mission::BASIC};
    std::atomic<bool> missionPossible_{true};
    std::mutex mutex_;  // Thread synchronization
    
    void startControllers();
    void updateMissionStatus();
};

#endif // MISSION_H