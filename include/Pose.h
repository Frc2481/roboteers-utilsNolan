#ifndef POSE_H
#define POSE_H

#include "RigidTransform2D.h"
#include "TankDriveKinematics.h"

class Pose {
public:
    Pose(const RigidTransform2D &pose, const double &wheelTrack);
    ~Pose();

    void setCornerStiffCoeff(const double &cornerStiffCoeff);

    //////////////////////////////////////////////////////////////////////
    // @brief reset pose
    //////////////////////////////////////////////////////////////////////
    void reset(const RigidTransform2D &pose);

    //////////////////////////////////////////////////////////////////////
    // @brief update pose estimate
    // @param deltaDistLeftWheel - change in left wheel linear distance (in)
    // @param deltaDistRightWheel - change in right wheel linear distance (in)
    // @param deltaYawGyro - change in gyro yaw (deg)
    //////////////////////////////////////////////////////////////////////
    RigidTransform2D update(
        const double &deltaDistLeftWheel,
        const double &deltaDistRightWheel,
        const double &deltaYawGyro);

private:
    RigidTransform2D m_pose;
    TankDriveKinematics m_kinematics;
    double m_cornerStiffCoeff;
};

#endif // POSE_H
