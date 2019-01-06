#ifndef COMMANDS_SWERVE_DRIVETRAIN_SHIFT_LOW_GEAR_H
#define COMMANDS_SWERVE_DRIVETRAIN_SHIFT_LOW_GEAR_H

#include <vector>
#include <limits>
#include "CommandBase.h"

class SwerveDrivetrainShiftLowGear : public CommandBase {
public:
	SwerveDrivetrainShiftLowGear()
		: CommandBase("SwerveDrivetrainShiftLowGear") {
		
		Requires(CommandBase::m_pSwerveDrivetrain.get());
		SetInterruptible(false);
	}

	~SwerveDrivetrainShiftLowGear() {
	}

	void Initialize() {
		m_pSwerveDrivetrain->setShiftState(false);
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

#endif // COMMANDS_SWERVE_DRIVETRAIN_SHIFT_LOW_GEAR_H
