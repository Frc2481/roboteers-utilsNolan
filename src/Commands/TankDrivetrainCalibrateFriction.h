#ifndef COMMANDS_TANK_DRIVETRAIN_CALIBRATE_FRICTION_H
#define COMMANDS_TANK_DRIVETRAIN_CALIBRATE_FRICTION_H

#include <vector>
#include <limits>
#include "CommandBase.h"
#include "WPILib.h"

class TankDrivetrainCalibrateFriction : public CommandBase {
public:
	TankDrivetrainCalibrateFriction()
		: CommandBase("TankDrivetrainCalibrateFriction") {
		
		double refV = 0;
		Requires(CommandBase::m_pTankDrivetrain.get());
		SetInterruptible(true);
		SmartDashboard::PutNumber("refV", refV);
	}

	~TankDrivetrainCalibrateFriction() {
	}

	void Initialize() {
	}

	void Execute() {
		double refV = SmartDashboard::GetNumber("refV", 0);
		CommandBase::m_pTankDrivetrain->driveClosedLoopControl(refV, 0, 0);
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

#endif // COMMANDS_TANK_DRIVETRAIN_CALIBRATE_FRICTION_H
