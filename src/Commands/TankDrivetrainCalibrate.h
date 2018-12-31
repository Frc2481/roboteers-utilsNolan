#ifndef COMMANDS_TANK_DRIVETRAIN_CALIBRATE_H
#define COMMANDS_TANK_DRIVETRAIN_CALIBRATE_H

#include <vector>
#include <limits>
#include "CommandBase.h"
#include "WPILib.h"

class TankDrivetrainCalibrate : public CommandBase {
public:
	TankDrivetrainCalibrate()
		: CommandBase("TankDrivetrainCalibrate") {
		
		Requires(CommandBase::m_pTankDrivetrain.get());
		SetInterruptible(true);
	}

	~TankDrivetrainCalibrate() {
	}

	void Initialize() {
	}

	void Execute() {
		double refVel = SmartDashboard::GetNumber("refVel", 0);
		double refYawRate = SmartDashboard::GetNumber("refYawRate", 0);
		double refAccel = SmartDashboard::GetNumber("refAccel", 0);
		double refYawAccel = SmartDashboard::GetNumber("refYawAccel", 0);
		CommandBase::m_pTankDrivetrain->driveClosedLoopControl(refVel, refYawRate, refAccel, refYawAccel);
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {
		return false;
	}

	void End() {
		m_pTankDrivetrain->stop();
	}

private:

};

#endif // COMMANDS_TANK_DRIVETRAIN_CALIBRATE_H
