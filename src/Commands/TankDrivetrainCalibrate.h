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

		SmartDashboard::PutNumber("refV", 0);
		SmartDashboard::PutNumber("refYawRate", 0);
		SmartDashboard::PutNumber("refA", 0);
	}

	~TankDrivetrainCalibrate() {
	}

	void Initialize() {
	}

	void Execute() {
		double refV = SmartDashboard::GetNumber("refV", 0);
		double refYawRate = SmartDashboard::GetNumber("refYawRate", 0);
		double refA = SmartDashboard::GetNumber("refA", 0);
		CommandBase::m_pTankDrivetrain->driveClosedLoopControl(refV, refYawRate, refA);
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
