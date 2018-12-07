#ifndef COMMANDS_TANK_DRIVETRAIN_CALIBRATE_FRICTION_H
#define COMMANDS_TANK_DRIVETRAIN_CALIBRATE_FRICTION_H

#include <vector>
#include <limits>
#include "CommandBase.h"

class TankDrivetrainCalibrateFriction : public CommandBase {
public:
	TankDrivetrainCalibrateFriction()
		: CommandBase("TankDrivetrainCalibrateFriction") {
		
		Requires(CommandBase::m_pTankDrivetrain.get());
		SetInterruptible(true);
	}

	~TankDrivetrainCalibrateFriction() {
	}

	void Initialize() {
	}

	void Execute() {
		CommandBase::m_pTankDrivetrain->driveClosedLoopControl(0, 0, 0);
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
