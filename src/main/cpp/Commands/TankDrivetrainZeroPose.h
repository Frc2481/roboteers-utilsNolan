#ifndef COMMANDS_TANK_DRIVETRAIN_ZERO_POSE_H
#define COMMANDS_TANK_DRIVETRAIN_ZERO_POSE_H

#include <vector>
#include <limits>
#include "../CommandBase.h"

class TankDrivetrainZeroPose : public CommandBase {
public:
	TankDrivetrainZeroPose()
		: CommandBase("TankDrivetrainZeroPose") {
		
		Requires(CommandBase::m_pTankDrivetrain.get());
		SetInterruptible(false);
	}

	~TankDrivetrainZeroPose() {
	}

	void Initialize() {
		m_pTankDrivetrain->resetPose(Pose2D(Translation2D(0, 0), Rotation2D::fromDegrees(0)), PoseDot2D(0, 0, 0));
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

#endif // COMMANDS_TANK_DRIVETRAIN_ZERO_POSE_H
