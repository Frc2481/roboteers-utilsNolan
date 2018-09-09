#ifndef TANK_DRIVE_POSE_H
#define TANK_DRIVE_POSE_H

#include "Pose2D.h"
#include "TankDriveKinematics.h"

// +x = robot right
// +y = robot forward
// +yaw = CCW, zero is robot forward

class TankDrivePose {
public:
    TankDrivePose(const Pose2D &pose, const double &wheelTrack);
    ~TankDrivePose();

    void setCornerStiffCoeff(const double &cornerStiffCoeff);

    //////////////////////////////////////////////////////////////////////
    // @brief reset pose
    //////////////////////////////////////////////////////////////////////
    void reset(const Pose2D &pose);

    //////////////////////////////////////////////////////////////////////
    // @brief update pose estimate
    // @param deltaDistLeftWheel - change in left wheel linear distance (in)
    // @param deltaDistRightWheel - change in right wheel linear distance (in)
    // @param deltaYawGyro - change in gyro yaw (deg)
    //////////////////////////////////////////////////////////////////////
    Pose2D update(
        const double &deltaDistLeftWheel,
        const double &deltaDistRightWheel,
        const double &deltaYawGyro);

private:
    Pose2D m_pose;
    TankDriveKinematics m_kinematics;
    double m_cornerStiffCoeff;
};

#endif // TANK_DRIVE_POSE_H
