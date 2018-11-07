#ifndef COMMANDS_TANK_DRIVETRAIN_ROTATE_TO_ANGLE_H
#define COMMANDS_TANK_DRIVETRAIN_ROTATE_TO_ANGLE_H

#include <vector>
#include <limits>
#include "CommandBase.h"
#include "Subsystems/TankDrivetrain.h"
#include "PIDVAController.h"
#include "RobotParameters.h"

class TankDrivetrainRotateToAngle : public CommandBase {
public:
	TankDrivetrainRotateToAngle()
		: CommandBase("TankDrivetrainRotateToAngle"),
		m_refAng(0),
		m_time(0) {
		
		Requires(CommandBase::m_pTankDrivetrain.get());
		setInterruptible(true);

		PIDVAController::PIDVAController rotateController(
			RobotParameters::k_rotateToAngleControllerKp,
			RobotParameters::k_rotateToAngleControllerKi,
			RobotParameters::k_rotateToAngleControllerKd,
			RobotParameters::k_rotateToAngleControllerKv,
			RobotParameters::k_rotateToAngleControllerKa);
		rotateController.setIsContinous(true, -180, 180);
		rotateController.setTargetZone(RobotParameters::k_rotateToAngleControllerTargetZone);
		rotateController.setTargetZoneDebounce(RobotParameters::k_rotateToAngleControllerTargetZoneDebounce);
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
