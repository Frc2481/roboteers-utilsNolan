#ifndef COMMANDS_TANK_DRIVETRAIN_JOYSTICK_DRIVE_H
#define COMMANDS_TANK_DRIVETRAIN_JOYSTICK_DRIVE_H

#include <vector>
#include <limits>
#include "CommandBase.h"
#include "Subsystems/TankDrivetrain.h"
#include "Components/Joystick2481.h"
#include "XboxController.h"

class TankDrivetrainJoystickDrive : public CommandBase {
public:
	TankDrivetrainJoystickDrive()
		: CommandBase("TankDrivetrainJoystickDrive") {
		
		Requires(CommandBase::m_pTankDrivetrain.get());
		setInterruptible(true);
	}

	~TankDrivetrainJoystickDrive() {
	}

	void Initialize() {
	}

	void Execute() {
		// get joystick input
		double leftIn = oi->GetDriverStick()->GetRawAxis(XB_LEFT_Y_AXIS)
		double rightIn = oi->GetDriverStick()->GetRawAxis(XB_RIGHT_Y_AXIS)

		double robotVel = kp * (leftIn + rightIn) / 2.0
		double robotYawRate = kp * (rightIn - leftIn) / 2.0

		// update drive
		m_pTankDrivetrain->drive(robotVel, robotYawRate, 0);
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
