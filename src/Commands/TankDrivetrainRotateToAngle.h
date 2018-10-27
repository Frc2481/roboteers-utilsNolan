#ifndef COMMANDS_TANK_DRIVETRAIN_ROTATE_TO_ANGLE_H
#define COMMANDS_TANK_DRIVETRAIN_ROTATE_TO_ANGLE_H

#include <vector>
#include <limits>
#include "CommandBase.h"
#include "Subsystems/TankDrivetrain.h"
#include "PIDVAController.h"

class TankDrivetrainRotateToAngle : public CommandBase {
public:
	TankDrivetrainRotateToAngle()
		: CommandBase("TankDrivetrainRotateToAngle"),
		m_refAng(0),
		m_time(0) {
		
		Requires(CommandBase::m_pTankDrivetrain.get());
		setInterruptible(true);

		PIDVAController::PIDVAController rotateController(1, 0, 0, 0, 0);
		rotateController.setIsContinous(true, -180, 180);
		rotateController.setTargetZone(1);
		rotateController.setTargetZoneDebounce(1);
	}

	~TankDrivetrainRotateToAngle() {
	}

	void Initialize(refAngle) {
		m_refAng = refAngle;
		m_time = 0;
		rotateController.reset(m_time);
	}

	void Execute() {
		// update controller
		Pose2D pose = m_pTankDrivetrain->getPose();
		m_time += robotSechdulerPeriod;
		double robotYawRate;
		rotateController.update(refAngle, 0, 0, pose.getRotation().getDegrees(), m_time, robotYawRate);

		// update drive
		m_pTankDrivetrain->driveClosedLoopControl(0, robotYawRate, 0);
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {
		return rotateController.isOnTarget();
	}

	void End() {
		m_pTankDrivetrain->stop();
	}

private:
	m_time;
};

#endif // COMMANDS_TANK_DRIVETRAIN_ROTATE_TO_ANGLE_H
