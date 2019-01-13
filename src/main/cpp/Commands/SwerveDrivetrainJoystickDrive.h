// #ifndef COMMANDS_SWERVE_DRIVETRAIN_JOYSTICK_DRIVE_H
// #define COMMANDS_SWERVE_DRIVETRAIN_JOYSTICK_DRIVE_H

// #include <vector>
// #include <limits>
// #include "../CommandBase.h"
// #include "../Components/Joystick2481.h"
// #include "../Components/XboxController2481.h"

// class SwerveDrivetrainJoystickDrive : public CommandBase {
// public:
// 	SwerveDrivetrainJoystickDrive()
// 		: CommandBase("SwerveDrivetrainJoystickDrive") {
		
// 		Requires(CommandBase::m_pSwerveDrivetrain.get());
// 		SetInterruptible(true);
// 	}

// 	~SwerveDrivetrainJoystickDrive() {
// 	}

// 	void Initialize() {
// 	}

// 	void Execute() {
// 		// get joystick input
// 		double percentVelX = CommandBase::m_pOI->GetDriverStick()->GetRawAxis(XBOX_LEFT_X_AXIS);
// 		double percentVelY = -CommandBase::m_pOI->GetDriverStick()->GetRawAxis(XBOX_LEFT_Y_AXIS);
// 		double percentYawRate = -CommandBase::m_pOI->GetDriverStick()->GetRawAxis(XBOX_RIGHT_Y_AXIS);

// 		// update drive
// 		m_pSwerveDrivetrain->driveOpenLoopControl(percentVelX, percentVelY, percentYawRate);
// 	}

// 	void Interrupted() {
// 		End();
// 	}

// 	bool IsFinished() {
// 		return false;
// 	}

// 	void End() {
// 		m_pSwerveDrivetrain->stop();
// 	}

// private:

// };

// #endif // COMMANDS_SWERVE_DRIVETRAIN_JOYSTICK_DRIVE_H
