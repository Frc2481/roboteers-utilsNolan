#ifndef COMMANDS_TANK_DRIVETRAIN_FOLLOW_PATH_H
#define COMMANDS_TANK_DRIVETRAIN_FOLLOW_PATH_H

#include <vector>
#include <limits>
#include "CommandBase.h"
#include "Utils/TankDrivePathGenerator.h"
#include "Utils/Translation2D.h"
#include "RobotParameters.h"
#include "Utils/Sign.h"

class TankDrivetrainFollowPath : public CommandBase {
public:
	TankDrivetrainFollowPath(
		std::vector<TankDrivePathGenerator::waypoint_t> &waypoints,
		bool isReverse,
		double targetZone)

		: CommandBase("TankDrivetrainFollowPath"),
		m_lastPointReached(false),
		m_distToEnd(std::numeric_limits<double>::infinity()),
		m_targetZone(targetZone) {
		
		Requires(CommandBase::m_pTankDrivetrain.get());
		SetInterruptible(true);

		// generate path
		TankDrivePathGenerator pathGenerator(
			waypoints,
			RobotParameters::k_updateRate,
			RobotParameters::k_wheelTrack,
			RobotParameters::k_maxSpeed,
			RobotParameters::k_maxAccel,
			RobotParameters::k_maxDeccel,
			RobotParameters::k_maxCentripAccel);
		pathGenerator.setIsReverse(isReverse);
		pathGenerator.generatePath();
		m_path = pathGenerator.getFinalPath();
		m_path.erase(m_path.begin()); // fix this workaround of vel and accel zero at start

		// ensure target zone is achievable
		if(m_targetZone < 1)
		{
			m_targetZone = 1;
		}
	}

	~TankDrivetrainFollowPath() {
	}

	void Initialize() {
		SetTimeout(m_path.end()->time + RobotParameters::k_pathFollowerTimeoutAllowance);
	}

	void Execute() {
		// update pose
		Pose2D pose = m_pTankDrivetrain->getPose();

		// find closest point on path
		if(m_path.empty()) {
			m_lastPointReached = true;
			m_distToEnd = 0;
			return;
		}

		Translation2D vectRobotToClosestPoint;
		double distToClosestPoint = std::numeric_limits<double>::infinity();
		std::vector<TankDrivePathGenerator::finalPathPoint_t>::iterator closestPointIt;
		// could start search at last closest point for efficiency gain but decided not to in case
		// pose jumps backward along path and want to start tracking path from new point on path.
		// could end search after a few iterations for efficiency gain but decided not to in case
		// pose jumps forward along path and want to start tracking path from new point on path.
		for(std::vector<TankDrivePathGenerator::finalPathPoint_t>::iterator it = m_path.begin(); it != m_path.end(); ++it) {
			Translation2D vectRobotToPoint = Translation2D(it->xPos, it->yPos) - pose.getTranslation();
			double distToPoint = vectRobotToPoint.norm();
			if(distToPoint < distToClosestPoint) {
				vectRobotToClosestPoint = vectRobotToPoint;
				distToClosestPoint = distToPoint;
				closestPointIt = it;
			}
		}

		// check if last point reached
		if(closestPointIt == m_path.end()) {
			m_lastPointReached  = true;
		}

		// calculate distance to end of path
		m_distToEnd = (Translation2D(m_path.back().xPos, m_path.back().yPos) - pose.getTranslation()).norm();

		// control law
		double robotVel = closestPointIt->vel;
		double robotAccel = closestPointIt->accel;
		double robotYawRate = closestPointIt->yawRate;
		// need to get direction vector to determine if left or right of path
		Translation2D vectClosestPointToNextPoint = Translation2D(1, 0).rotateBy(Rotation2D::fromDegrees(closestPointIt->yaw));
		int leftRightOfPath = Sign::sign(vectClosestPointToNextPoint.cross(vectRobotToClosestPoint));
		robotYawRate -= RobotParameters::k_pathFollowerKpTurn * leftRightOfPath * distToClosestPoint; // assume this is perpindicular distance to path if it is closest point on path

		// update drive
		m_pTankDrivetrain->driveClosedLoopControl(robotVel, robotYawRate, robotAccel);
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {
		return (std::fabs(m_distToEnd) < m_targetZone) || m_lastPointReached;
	}

	void End() {
		m_pTankDrivetrain->stop();
	}

private:
	std::vector<TankDrivePathGenerator::finalPathPoint_t> m_path;
	bool m_lastPointReached;
	double m_distToEnd;
	double m_targetZone;
};

#endif // COMMANDS_TANK_DRIVETRAIN_FOLLOW_PATH_H
