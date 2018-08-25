#include "TrajectoryGenerator1D.h"
#include <iostream>

int main() {
    std::vector<TrajectoryGenerator1D::waypoint_t> waypoints;

    TrajectoryGenerator1D pathGenerator(waypoints, 100, 100, 100, -100);
    TrajectoryGenerator1D::waypoint_t tempWaypoint;
    
    pathGenerator.readWaypointsFromCSV();
    pathGenerator.generatePath();
    pathGenerator.writePathToCSV();
//    pathGenerator.writeComboPathToCSV();
//    pathGenerator.writeTempPathToCSV();
}