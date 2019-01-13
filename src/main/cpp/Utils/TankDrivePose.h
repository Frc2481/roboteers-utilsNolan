#ifndef TANK_DRIVE_POSE_H
#define TANK_DRIVE_POSE_H

#include "Pose2D.h"
#include "PoseDot2D.h"
#include "TankDriveKinematics.h"

// +x = field right
// +y = field forward
// +yaw = CCW, zero is field forward

class TankDrivePose {
public:
    TankDrivePose(
        const Pose2D &pose,
        double wheelTrack,
        double cornerStiffCoeff);
    ~TankDrivePose();

    void reset(const Pose2D &pose, const PoseDot2D &poseDot);
    Pose2D getPose();
    PoseDot2D getPoseDot();

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
        double deltaDistLeftWheel,
        double deltaDistRightWheel,
        double deltaYawGyro,
        double velLeftWheel,
        double velRightWheel,
        double yawRateGyro);

private:
    Pose2D m_pose;
    PoseDot2D m_poseDot;
    TankDriveKinematics m_kinematics;
    double m_cornerStiffCoeff;
};

#endif // TANK_DRIVE_POSE_H
