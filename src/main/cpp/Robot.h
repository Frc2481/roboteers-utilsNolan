#ifndef ROBOT_H
#define ROBOT_H

#include <frc/WPILib.h>

class Robot : public frc::TimedRobot {
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
};

#endif // ROBOT_H
