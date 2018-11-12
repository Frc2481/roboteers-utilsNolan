#ifndef ROBOT_H
#define ROBOT_H

#include <WPILib.h>

class Robot : public TimedRobot {
public:
	void RobotInit() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

private:
	std::unique_ptr<Command> m_pAutonomousCommand;
};

#endif // ROBOT_H
