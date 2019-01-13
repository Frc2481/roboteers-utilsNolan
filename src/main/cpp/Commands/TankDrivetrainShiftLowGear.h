#ifndef COMMANDS_TANK_DRIVETRAIN_SHIFT_LOW_GEAR_H
#define COMMANDS_TANK_DRIVETRAIN_SHIFT_LOW_GEAR_H

#include <vector>
#include <limits>
#include "../CommandBase.h"

class TankDrivetrainShiftLowGear : public CommandBase {
public:
	TankDrivetrainShiftLowGear()
		: CommandBase("TankDrivetrainShiftLowGear") {
		
		Requires(CommandBase::m_pTankDrivetrain.get());
		SetInterruptible(false);
	}

	~TankDrivetrainShiftLowGear() {
	}

	void Initialize() {
		m_pTankDrivetrain->setShiftState(false);
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

#endif // COMMANDS_TANK_DRIVETRAIN_SHIFT_LOW_GEAR_H
