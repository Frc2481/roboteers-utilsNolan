#ifndef COMMANDS_TANK_DRIVETRAIN_ROTATE_TO_ANGLE_H
#define COMMANDS_TANK_DRIVETRAIN_ROTATE_TO_ANGLE_H

#include <vector>
#include <limits>
#include "../CommandBase.h"
#include "../RobotParameters.h"
#include "../Utils/TrajectoryGenerator1D.h"

class TankDrivetrainRotateToAngle : public CommandBase {
public:
	TankDrivetrainRotateToAngle(double refAngle)

		: CommandBase("TankDrivetrainRotateToAngle"),
		m_lastPointReached(false),
		m_distToEnd(std::numeric_limits<double>::infinity()) {

		Requires(CommandBase::m_pTankDrivetrain.get());
		SetInterruptible(true);

		Pose2D pose = m_pTankDrivetrain->getPose();
		PoseDot2D poseDot = m_pTankDrivetrain->getPoseDot();
		std::vector<TrajectoryGenerator1D::waypoint_t> waypoints;
		waypoints.push_back(TrajectoryGenerator1D::waypoint_t {pose.getRotation().getDegrees(), poseDot.getYawRate()});
		waypoints.push_back(TrajectoryGenerator1D::waypoint_t {refAngle, 0});

		TrajectoryGenerator1D trajectoryGenerator1D(
			waypoints,
			RobotParameters::k_updateRate,
			RobotParameters::k_rotateToAngleMaxYawRate,
			RobotParameters::k_rotateToAngleMaxYawAccel,
			RobotParameters::k_rotateToAngleMaxYawDeccel);
		trajectoryGenerator1D.setIsContinous(true, -180, 180);
		trajectoryGenerator1D.generatePath();
		m_path = trajectoryGenerator1D.getFinalPath();
		m_path.erase(m_path.begin()); // workaround of vel and accel zero at start

		SetTimeout(m_path.end()->time + RobotParameters::k_rotateToAngleTimeoutAllowance);
	}

	~TankDrivetrainRotateToAngle() {
	}

	void Initialize() {
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

		double distToClosestPoint = std::numeric_limits<double>::infinity();
		std::vector<TrajectoryGenerator1D::finalPathPoint_t>::iterator closestPointIt;
		// could start search at last closest point for efficiency gain but decided not to in case
		// pose jumps backward along path and want to start tracking path from new point on path.
		// could end search after a few iterations for efficiency gain but decided not to in case
		// pose jumps forward along path and want to start tracking path from new point on path.
		for(std::vector<TrajectoryGenerator1D::finalPathPoint_t>::iterator it = m_path.begin(); it != m_path.end(); ++it) {
			double distToPoint = fabs(it->pos - pose.getRotation().getDegrees());
			if(distToPoint < distToClosestPoint) {
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
		double robotYawRate = closestPointIt->vel;
		double robotYawAccel = closestPointIt->accel;

		// update drive
		m_pTankDrivetrain->driveClosedLoopControl(0, robotYawRate, 0, robotYawAccel);
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {
		return IsTimedOut() || (m_distToEnd < RobotParameters::k_rotateToAngleTargetZone) || m_lastPointReached;
	}

	void End() {
		m_pTankDrivetrain->stop();
	}

private:
	std::vector<TrajectoryGenerator1D::finalPathPoint_t> m_path;
	bool m_lastPointReached;
	double m_distToEnd;
};

#endif // COMMANDS_TANK_DRIVETRAIN_ROTATE_TO_ANGLE_H
