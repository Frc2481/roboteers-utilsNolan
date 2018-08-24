#include "TrajectoryGenerator1D.h"
#include <iostream>

int main() {
    TrajectoryGenerator1D pathGenerator;
    std::vector<TrajectoryGenerator1D::waypoint> waypoints;
    TrajectoryGenerator1D::waypoint tempWaypoint;
    
    pathGenerator.readWaypointsFromCSV();
    pathGenerator.generatePath();
    pathGenerator.writePathToCSV();
//    pathGenerator.writeComboPathToCSV();
//    pathGenerator.writeTempPathToCSV();
}