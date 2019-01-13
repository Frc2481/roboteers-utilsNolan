#ifndef SWERVE_DRIVE_POSE_H
#define SWERVE_DRIVE_POSE_H

#include "Pose2D.h"
#include "PoseDot2D.h"
#include "SwerveDriveKinematics.h"

// +x = field right
// +y = field forward
// +yaw = CCW, zero is field forward

class SwerveDrivePose {
public:
	SwerveDrivePose(
        const Pose2D &pose,
        double wheelTrack,
		double wheelBase,
        double cornerStiffCoeff);
    ~SwerveDrivePose();

    void reset(const Pose2D &pose, const PoseDot2D &poseDot);
    Pose2D getPose();
    PoseDot2D getPoseDot();

    //////////////////////////////////////////////////////////////////////
    // @brief update pose estimate
    // @param deltaDistFRWheel - change in front right wheel linear distance (in)
    // @param deltaDistBRWheel - change in back right wheel linear distance (in)
    // @param deltaDistBLWheel - change in back left wheel linear distance (in)
    // @param deltaDistFLWheel - change in front left wheel linear distance (in)
    // @param deltaYawGyro - change in gyro yaw (deg)
    // @param velFRWheel - front right wheel linear velocity (in/s)
    // @param velBRWheel - back right wheel linear velocity (in/s)
    // @param velBLWheel - back left wheel linear velocity (in/s)
    // @param velFLWheel - front left wheel linear velocity (in/s)
    // @param yawRateGyro - yaw rate gyro (deg/s)
    //////////////////////////////////////////////////////////////////////
    void update(
        double deltaDistFRWheel,
		double deltaDistBRWheel,
		double deltaDistBLWheel,
		double deltaDistFLWheel,
		Rotation2D frWheelYaw,
		Rotation2D brWheelYaw,
		Rotation2D blWheelYaw,
		Rotation2D flWheelYaw,
        double deltaYawGyro,
        double velFRWheel,
        double velBRWheel,
		double velBLWheel,
		double velFLWheel,
        double yawRateGyro);

private:
    Pose2D m_pose;
    PoseDot2D m_poseDot;
    SwerveDriveKinematics m_kinematics;
    double m_cornerStiffCoeff;
};

#endif // SWERVE_DRIVE_POSE_H
