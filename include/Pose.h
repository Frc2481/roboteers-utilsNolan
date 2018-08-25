#ifndef POSE_H
#define POSE_H

#include "RigidTransform2D.h"
#include "TankDriveKinematics.h"

class Pose {
public:
    Pose(const RigidTransform2D &pose, const double &wheelTrack);
    ~Pose();

    void reset(const RigidTransform2D &pose);

    RigidTransform2D update(
        const double &deltaDistLeftWheel,
        const double &deltaDistRightWheel,
        const double &deltaYawGyro);

private:
    RigidTransform2D m_pose;
    TankDriveKinematics m_kinematics;
};

#endif // POSE_H
