#include "WPILib.h"
#include <TimedRobot.h>
#include "CommandBase.h"
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot: public TimedRobot {
public:

private:
    void RobotInit() {
        SetPeriod(ROBOT_SCHEDULER_PERIOD);
        CommandBase::init();
    }

    void AutonomousInit() override {
    }
    
    void DisabledInit() override {
    }
    
    void TeleopInit() override {
        if (autonomousCommand != nullptr) {
            autonomousCommand->Cancel();
        }
    }

    void AutonomousPeriodic() override {
        frc::Scheduler::GetInstance()->Run();
    }

    void TeleopPeriodic() override {
        frc::Scheduler::GetInstance()->Run();
    }
    
    void DisabledPeriodic() override {
        frc::Scheduler::GetInstance()->Run();
    }

    void TestPeriodic() override {
        frc::LiveWindow::GetInstance()->Run();
    }
};

START_ROBOT_CLASS(Robot)
