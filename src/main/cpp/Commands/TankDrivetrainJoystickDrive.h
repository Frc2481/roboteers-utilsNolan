#ifndef COMMANDS_TANK_DRIVETRAIN_JOYSTICK_DRIVE_H
#define COMMANDS_TANK_DRIVETRAIN_JOYSTICK_DRIVE_H

#include <vector>
#include <limits>
#include "../CommandBase.h"
#include "../Components/Joystick2481.h"
#include "../Components/XboxController2481.h"

class TankDrivetrainJoystickDrive : public CommandBase {
public:
	TankDrivetrainJoystickDrive()
		: CommandBase("TankDrivetrainJoystickDrive") {
		
		Requires(CommandBase::m_pTankDrivetrain.get());
		SetInterruptible(true);
	}

	~TankDrivetrainJoystickDrive() {
	}

	void Initialize() {
	}

	void Execute() {
		// get joystick input
		double percentLeftDrive = -CommandBase::m_pOI->GetDriverStick()->GetRawAxis(XBOX_LEFT_Y_AXIS);
		double percentRightDrive = -CommandBase::m_pOI->GetDriverStick()->GetRawAxis(XBOX_RIGHT_Y_AXIS);

		// update drive
		m_pTankDrivetrain->driveOpenLoopControl(percentLeftDrive, percentRightDrive);
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

#endif // COMMANDS_TANK_DRIVETRAIN_JOYSTICK_DRIVE_H
