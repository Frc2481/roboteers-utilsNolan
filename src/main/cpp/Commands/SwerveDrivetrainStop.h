// #ifndef COMMANDS_SWERVE_DRIVETRAIN_STOP_H
// #define COMMANDS_SWERVE_DRIVETRAIN_STOP_H

// #include <vector>
// #include <limits>
// #include "../CommandBase.h"

// class SwerveDrivetrainStop : public CommandBase {
// public:
// 	SwerveDrivetrainStop()
// 		: CommandBase("SwerveDrivetrainStop") {
		
// 		Requires(CommandBase::m_pSwerveDrivetrain.get());
// 		SetInterruptible(false);
// 	}

// 	~SwerveDrivetrainStop() {
// 	}

// 	void Initialize() {
// 		m_pSwerveDrivetrain->stop();
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

// #endif // COMMANDS_SWERVE_DRIVETRAIN_STOP_H
