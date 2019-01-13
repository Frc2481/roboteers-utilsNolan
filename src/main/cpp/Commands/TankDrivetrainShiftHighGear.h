#ifndef COMMANDS_TANK_DRIVETRAIN_SHIFT_HIGH_GEAR_H
#define COMMANDS_TANK_DRIVETRAIN_SHIFT_HIGH_GEAR_H

#include <vector>
#include <limits>
#include "../CommandBase.h"

class TankDrivetrainShiftHighGear : public CommandBase {
public:
	TankDrivetrainShiftHighGear()
		: CommandBase("TankDrivetrainShiftHighGear") {
		
		Requires(CommandBase::m_pTankDrivetrain.get());
		SetInterruptible(false);
	}

	~TankDrivetrainShiftHighGear() {
	}

	void Initialize() {
		m_pTankDrivetrain->setShiftState(true);
	}

	void Execute() {
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {
		return true;
	}

	void End() {
	}

private:

};

#endif // COMMANDS_TANK_DRIVETRAIN_SHIFT_HIGH_GEAR_H
