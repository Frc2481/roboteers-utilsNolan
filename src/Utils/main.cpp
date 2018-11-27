//#include "TankDrivePathGenerator.h"
//#include <iostream>
//
//int main() {
//    std::vector<TankDrivePathGenerator::waypoint_t> waypoints;
//    TankDrivePathGenerator::waypoint_t waypoint;
//
//    waypoint = {50, 20, 0, 0};
//    waypoints.push_back(waypoint);
//
//    waypoint = {50, 240, 1000, 20};
//    waypoints.push_back(waypoint);
//
//    waypoint = {240, 240, 1000, 20};
//    waypoints.push_back(waypoint);
//
//    waypoint = {240, 280, 0, 0};
//    waypoints.push_back(waypoint);
//
//    TankDrivePathGenerator pathGenerator(waypoints, 20, 20, 150, 150, -150, 30);
//    pathGenerator.setIsReverse(true);
//
//    // pathGenerator.readWaypointsFromCSV();
//    pathGenerator.generatePath();
//    pathGenerator.writePathToCSV();
//    // pathGenerator.writeComboPathToCSV();
//    // pathGenerator.writeTempPathToCSV();
//}
