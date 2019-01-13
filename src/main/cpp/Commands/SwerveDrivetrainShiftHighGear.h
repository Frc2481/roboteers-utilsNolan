// #ifndef COMMANDS_SWERVE_DRIVETRAIN_SHIFT_HIGH_GEAR_H
// #define COMMANDS_SWERVE_DRIVETRAIN_SHIFT_HIGH_GEAR_H

// #include <vector>
// #include <limits>
// #include "../CommandBase.h"

// class SwerveDrivetrainShiftHighGear : public CommandBase {
// public:
// 	SwerveDrivetrainShiftHighGear()
// 		: CommandBase("SwerveDrivetrainShiftHighGear") {
		
// 		Requires(CommandBase::m_pSwerveDrivetrain.get());
// 		SetInterruptible(false);
// 	}

// 	~SwerveDrivetrainShiftHighGear() {
// 	}

// 	void Initialize() {
// 		m_pSwerveDrivetrain->setShiftState(true);
// 	}

// 	void Execute() {
// 	}

// 	void Interrupted() {
// 		End();
// 	}

// 	bool IsFinished() {
// 		return true;
// 	}

// 	void End() {
// 	}

// private:

// };

// #endif // COMMANDS_SWERVE_DRIVETRAIN_SHIFT_HIGH_GEAR_H
