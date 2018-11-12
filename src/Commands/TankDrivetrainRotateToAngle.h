#ifndef COMMANDS_TANK_DRIVETRAIN_ROTATE_TO_ANGLE_H
#define COMMANDS_TANK_DRIVETRAIN_ROTATE_TO_ANGLE_H

#include <vector>
#include <limits>
#include "CommandBase.h"
#include "PIDVAController.h"
#include "RobotParameters.h"

class TankDrivetrainRotateToAngle : public CommandBase {
public:
	TankDrivetrainRotateToAngle()
		: CommandBase("TankDrivetrainRotateToAngle"),
		m_refAng(0),
		m_time(0) {
		
		Requires(CommandBase::m_pTankDrivetrain.get());
		SetInterruptible(true);

		m_pRotateController = new PIDVAController(
			RobotParameters::k_rotateToAngleControllerKp,
			RobotParameters::k_rotateToAngleControllerKi,
			RobotParameters::k_rotateToAngleControllerKd,
			RobotParameters::k_rotateToAngleControllerKv,
			RobotParameters::k_rotateToAngleControllerKa);
		m_pRotateController->setIsContinous(true, -180, 180);
		m_pRotateController->setTargetZone(RobotParameters::k_rotateToAngleControllerTargetZone);
		m_pRotateController->setTargetZoneDebounce(RobotParameters::k_rotateToAngleControllerTargetZoneDebounce);
	}

	~TankDrivetrainRotateToAngle() {
	}

	void Initialize(const double &refAngle) {
		m_refAng = refAngle;
		m_time = 0;
		m_pRotateController->reset(m_time);
	}

	void Execute() {
		// update controller
		Pose2D pose = m_pTankDrivetrain->getPose();
		m_time += 1.0 / (double)RobotParameters::k_updateRate;
		double robotYawRate;
		m_pRotateController->update(m_refAng, 0, 0, pose.getRotation().getDegrees(), m_time, robotYawRate);

		// update drive
		m_pTankDrivetrain->driveClosedLoopControl(0, robotYawRate, 0);
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {
		return m_pRotateController->isOnTarget();
	}

	void End() {
		m_pTankDrivetrain->stop();

		delete m_pTankDrivetrain;
		m_pTankDrivetrain = nullptr;
	}

private:
	double m_refAng;
	double m_time;
	PIDVAController* m_pRotateController;
};

#endif // COMMANDS_TANK_DRIVETRAIN_ROTATE_TO_ANGLE_H
