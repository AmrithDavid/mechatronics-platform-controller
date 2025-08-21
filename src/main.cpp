#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include "ackerman.h"
#include "quadcopter.h"
#include "mission.h"

int main(int argc, char *argv[]) {
    // Create platform controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());
    controllers.push_back(new Ackerman());
    
    // Set tolerance for goal achievement
    for (auto controller : controllers) {
        controller->setTolerance(0.5);
    }
    
    // Create mission coordinator
    Mission mission(controllers);
    mission.setMissionObjective(mission::Objective::BASIC);
    
    // Example goals for demonstration
    std::vector<pfms::geometry_msgs::Point> quadcopterGoals = {
        {5.0, 5.0, 0.0},
        {10.0, 5.0, 0.0},
        {10.0, 10.0, 0.0}
    };
    
    std::vector<pfms::geometry_msgs::Point> ackermanGoals = {
        {3.0, 3.0, 0.0},
        {6.0, 3.0, 0.0},
        {6.0, 6.0, 0.0}
    };
    
    // Assign goals to platforms
    mission.setGoals(quadcopterGoals, pfms::QUADCOPTER);
    mission.setGoals(ackermanGoals, pfms::ACKERMAN);
    
    // Start mission (non-blocking)
    mission.run();
    
    // Monitor progress
    bool missionComplete = false;
    while (!missionComplete) {
        std::vector<unsigned int> progress = mission.status();
        
        std::cout << "Mission Progress - ";
        std::cout << "Quadcopter: " << progress[0] << "% ";
        std::cout << "Ackerman: " << progress[1] << "%" << std::endl;
        
        if (progress[0] == 100 && progress[1] == 100) {
            missionComplete = true;
            std::cout << "Mission Complete!" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    // Cleanup
    for (auto controller : controllers) {
        delete controller;
    }
    
    return 0;
}