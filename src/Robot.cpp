#include "Robot.h"
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "CommandBase.h"
#include "RobotParameters.h"

void Robot::RobotInit() {
	SetPeriod(1.0 / RobotParameters::k_updateRate);
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

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
	CommandBase::m_pTankDrivetrain->Periodic();
}

void Robot::TestPeriodic() {
//	LiveWindow::GetInstance()->Run();
}

START_ROBOT_CLASS(Robot)
