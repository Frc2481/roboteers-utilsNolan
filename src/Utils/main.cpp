#include "TankDrivePathGenerator.h"
#include <iostream>

int main() {
    std::vector<TankDrivePathGenerator::waypoint_t> waypoints;

    TankDrivePathGenerator pathGenerator(waypoints, 0, 0, 0, 0, 0, 0);
    TankDrivePathGenerator::waypoint_t tempWaypoint;
    
    pathGenerator.readWaypointsFromCSV();
    pathGenerator.generatePath();
    pathGenerator.writePathToCSV();
    // pathGenerator.writeComboPathToCSV();
    // pathGenerator.writeTempPathToCSV();
}