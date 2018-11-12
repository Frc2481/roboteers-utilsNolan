#include "Robot.h"
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "CommandBase.h"
#include "RobotParameters.h"

void Robot::RobotInit() {
	SetPeriod(RobotParameters::k_updateRate);
	CommandBase::init();
}

void Robot::AutonomousInit() {
}

void Robot::DisabledInit() {
}

void Robot::TeleopInit() {
	if (m_pAutonomousCommand != nullptr) {
		m_pAutonomousCommand->Cancel();
		m_pAutonomousCommand = nullptr;
	}
}

void AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
}

void TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
}

void DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
}

void TestPeriodic() {
	LiveWindow::GetInstance()->Run();
}

START_ROBOT_CLASS(Robot)
