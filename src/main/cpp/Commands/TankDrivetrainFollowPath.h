#ifndef COMMANDS_TANK_DRIVETRAIN_FOLLOW_PATH_H
#define COMMANDS_TANK_DRIVETRAIN_FOLLOW_PATH_H

#include <vector>
#include <limits>
#include <algorithm>
#include "../CommandBase.h"
#include "../RobotParameters.h"
#include "../Utils/TankDrivePathGenerator.h"
#include "../Utils/Translation2D.h"
#include "../Utils/Sign.h"
#include "../Utils/PIDVAController.h"

class TankDrivetrainFollowPath : public CommandBase {
public:
	TankDrivetrainFollowPath(
		std::vector<TankDrivePathGenerator::waypoint_t> &waypoints,
		bool isReverse,
		double targetZone)

		: CommandBase("TankDrivetrainFollowPath"),
		m_lastPointReached(false),
		m_distToEnd(std::numeric_limits<double>::infinity()),
		m_targetZone(targetZone),
		m_time(0),
		m_latDistController(
			RobotParameters::k_pathFollowerLatDistKp,
			RobotParameters::k_pathFollowerLatDistKi,
			RobotParameters::k_pathFollowerLatDistKd,
			0,
			0),
		m_yawRateController(
			RobotParameters::k_pathFollowerYawRateKp,
			RobotParameters::k_pathFollowerYawRateKi,
			RobotParameters::k_pathFollowerYawRateKd,
			0,
			0)
		{
		
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
		m_path.erase(m_path.begin()); // workaround of vel and accel zero at start

		// ensure target zone is achievable
		if(m_targetZone < 1)
		{
			m_targetZone = 1;
		}

		SetTimeout(m_path.end()->time + RobotParameters::k_pathFollowerTimeoutAllowance);
	}

	~TankDrivetrainFollowPath() {
	}

	void Initialize() {
	}

	void Execute() {
		// update pose
		Pose2D pose = m_pTankDrivetrain->getPose();
		PoseDot2D poseDot = m_pTankDrivetrain->getPoseDot();

		// find closest point on path
		if(m_path.empty()) {
			m_lastPointReached = true;
			m_distToEnd = 0;
			return;
		}

		Translation2D vectClosestPointToRobot;
		double distToClosestPoint = std::numeric_limits<double>::infinity();
		std::vector<TankDrivePathGenerator::finalPathPoint_t>::iterator closestPointIt;
		// could start search at last closest point for efficiency gain but decided not to in case
		// pose jumps backward along path and want to start tracking path from new point on path.
		// could end search after a few iterations for efficiency gain but decided not to in case
		// pose jumps forward along path and want to start tracking path from new point on path.
		for(std::vector<TankDrivePathGenerator::finalPathPoint_t>::iterator it = m_path.begin(); it != m_path.end(); ++it) {
			Translation2D vectPointToRobot = pose.getTranslation() - Translation2D(it->xPos, it->yPos);
			double distToPoint = vectPointToRobot.norm();
			if(distToPoint < distToClosestPoint) {
				vectClosestPointToRobot = vectPointToRobot;
				distToClosestPoint = distToPoint;
				closestPointIt = it;
			}
		}

		// check if last point reached
		if(closestPointIt == m_path.end()) {
			m_lastPointReached  = true;
		}

		// calculate distance to end of path
		m_distToEnd = fabs(m_path.back().dist - closestPointIt->dist);

		// path follower control law
		// feed forward
		double robotVel = closestPointIt->vel;
		double robotAccel = closestPointIt->accel;
		double robotYawRate = closestPointIt->yawRate;

		// lateral distance feedback
		double latDistCntrl = 0;
		m_time += 1.0 / RobotParameters::k_updateRate;
		Translation2D vectPathToRobot = vectClosestPointToRobot.rotateBy(Rotation2D::fromDegrees(-closestPointIt->yaw));
		m_latDistController.update(0, 0, 0, vectPathToRobot.getX(), m_time, latDistCntrl);
		robotYawRate -= latDistCntrl;

		// yaw rate feedback
		double yawRateCntrl = 0;
		m_yawRateController.update(closestPointIt->yawRate, 0, 0, poseDot.getYawRate(), m_time, yawRateCntrl);
		robotYawRate -= yawRateCntrl;

		// update drive
		m_pTankDrivetrain->driveClosedLoopControl(robotVel, robotYawRate, robotAccel, 0);
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {
		return IsTimedOut() || (m_distToEnd < m_targetZone) || m_lastPointReached;
	}

	void End() {
		m_pTankDrivetrain->stop();
	}

private:
	std::vector<TankDrivePathGenerator::finalPathPoint_t> m_path;
	bool m_lastPointReached;
	double m_distToEnd;
	double m_targetZone;
	double m_time;
	PIDVAController m_latDistController;
	PIDVAController m_yawRateController;
};

#endif // COMMANDS_TANK_DRIVETRAIN_FOLLOW_PATH_H
