#ifndef TANK_DRIVE_POSE_H
#define TANK_DRIVE_POSE_H

#include "Utils/Pose2D.h"
#include "Utils/TankDriveKinematics.h"

// +x = field right
// +y = field forward
// +yaw = CCW, zero is field forward

class TankDrivePose {
public:
    TankDrivePose(
        const Pose2D &pose,
        const double &wheelTrack,
        const double &cornerStiffCoeff);
    ~TankDrivePose();

    void reset(const Pose2D &pose, const Pose2D &poseDot);
    Pose2D getPose();
    Pose2D getPoseDot();

    //////////////////////////////////////////////////////////////////////
    // @brief update pose estimate
    // @param deltaDistLeftWheel - change in left wheel linear distance (in)
    // @param deltaDistRightWheel - change in right wheel linear distance (in)
    // @param deltaYawGyro - change in gyro yaw (deg)
    // @param velLeftWheel - left wheel linear velocity (in/s)
    // @param velRightWheel - right wheel linear velocity (in/s)
    // @param yawRateGyro - yaw rate gyro (deg/s)
    //////////////////////////////////////////////////////////////////////
    void update(
        const double &deltaDistLeftWheel,
        const double &deltaDistRightWheel,
        const double &deltaYawGyro,
        const double &velLeftWheel,
        const double &velRightWheel,
        const double &yawRateGyro);

private:
    Pose2D m_pose;
    Pose2D m_poseDot;
    TankDriveKinematics m_kinematics;
    double m_cornerStiffCoeff;
};

#endif // TANK_DRIVE_POSE_H
