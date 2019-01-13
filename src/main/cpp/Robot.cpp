#include "Robot.h"
#include "CommandBase.h"
#include "RobotParameters.h"
#include "Commands/TankDrivetrainZeroPose.h"
#include "Commands/TankDrivetrainFollowPath.h"
#include "Commands/TankDrivetrainCalibrate.h"
#include "Commands/TankDrivetrainRotateToAngle.h"

Robot::Robot() : TimedRobot(1.0 / RobotParameters::k_updateRate) {
}

void Robot::RobotInit() {
	CommandBase::Init();

	SmartDashboard::PutData("TankDrivetrainZeroPose", new TankDrivetrainZeroPose());

	std::vector<TankDrivePathGenerator::waypoint_t> waypoints;
	waypoints.push_back(TankDrivePathGenerator::waypoint_t {0, 0, 0, 0});
	waypoints.push_back(TankDrivePathGenerator::waypoint_t {0, 250, 0, 0});

	SmartDashboard::PutData("TankDrivetrainFollowPath", new TankDrivetrainFollowPath(waypoints, false, 5));

	SmartDashboard::PutData("TankDrivetrainCalibrate", new TankDrivetrainCalibrate());

	SmartDashboard::PutData("TankDrivetrainRotateToAngle", new TankDrivetrainRotateToAngle(180));
}

void Robot::AutonomousInit() {
}

void Robot::DisabledInit() {
}

void Robot::TeleopInit() {
}

void Robot::AutonomousPeriodic() {
	CommandBase::Periodic();
}

void Robot::TeleopPeriodic() {
	CommandBase::Periodic();
}

void Robot::DisabledPeriodic() {
	CommandBase::Periodic();
}

void Robot::TestPeriodic() {
	CommandBase::Periodic();
}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif
