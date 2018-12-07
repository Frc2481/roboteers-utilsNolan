#include "Robot.h"
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "CommandBase.h"
#include "RobotParameters.h"
#include "Commands/TankDrivetrainZeroPose.h"
#include "Commands/TankDrivetrainFollowPath.h"
#include "Commands/TankDrivetrainCalibrateFriction.h"

void Robot::RobotInit() {
	SetPeriod(1.0 / RobotParameters::k_updateRate);
	CommandBase::Init();

	SmartDashboard::PutData("TankDrivetrainZeroPose", new TankDrivetrainZeroPose());

	std::vector<TankDrivePathGenerator::waypoint_t> waypoints;
	waypoints.push_back(TankDrivePathGenerator::waypoint_t {0, 0, 0, 0});
	waypoints.push_back(TankDrivePathGenerator::waypoint_t {0, 100, 0, 0});
	SmartDashboard::PutData("TankDrivetrainFollowPath", new TankDrivetrainFollowPath(waypoints, false, 1));

	SmartDashboard::PutData("TankDrivetrainCalibrateFriction", new TankDrivetrainCalibrateFriction());
}

void Robot::AutonomousInit() {
}

void Robot::DisabledInit() {
}

void Robot::TeleopInit() {
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
	CommandBase::Periodic();
}

void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {
	Scheduler::GetInstance()->Run();
}

START_ROBOT_CLASS(Robot)
