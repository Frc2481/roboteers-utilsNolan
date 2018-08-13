#include "PathGenerator.h"
#include <iostream>

int main() {
    PathGenerator pathGenerator;
    std::vector<PathGenerator::waypoint> waypoints;
    PathGenerator::waypoint tempWaypoint;
    
    pathGenerator.readWaypointsFromCSV();
    pathGenerator.generatePath();
    pathGenerator.writePathToCSV();
//    pathGenerator.writeComboPathToCSV();
//    pathGenerator.writeTempPathToCSV();
}