// #ifndef COMMANDS_SWERVE_DRIVETRAIN_ZERO_POSE_H
// #define COMMANDS_SWERVE_DRIVETRAIN_ZERO_POSE_H

// #include <vector>
// #include <limits>
// #include "../CommandBase.h"

// class SwerveDrivetrainZeroPose : public CommandBase {
// public:
// 	SwerveDrivetrainZeroPose()
// 		: CommandBase("SwerveDrivetrainZeroPose") {
		
// 		Requires(CommandBase::m_pSwerveDrivetrain.get());
// 		SetInterruptible(false);
// 	}

// 	~SwerveDrivetrainZeroPose() {
// 	}

// 	void Initialize() {
// 		m_pSwerveDrivetrain->resetPose(Pose2D(Translation2D(0, 0), Rotation2D::fromDegrees(0)), PoseDot2D(0, 0, 0));
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

// #endif // COMMANDS_SWERVE_DRIVETRAIN_ZERO_POSE_H
