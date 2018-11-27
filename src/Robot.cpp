//#include "Robot.h"
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "CommandBase.h"
#include "RobotParameters.h"

class Robot : public TimedRobot {
public:

private:
void RobotInit() {
	SetPeriod(1.0 / RobotParameters::k_updateRate);
	CommandBase::init();
}

void AutonomousInit() {
}

void DisabledInit() {
}

void TeleopInit() {
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
	CommandBase::m_pTankDrivetrain->Periodic();
}

void TestPeriodic() {
//	LiveWindow::GetInstance()->Run();
}

std::unique_ptr<Command> m_pAutonomousCommand;
};

START_ROBOT_CLASS(Robot)
