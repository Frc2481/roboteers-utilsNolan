#include "Pose.h"

Pose::Pose(const RigidTransform2D &pose, const double &wheelTrack)
    : m_pose(pose),
    m_kinematics(wheelTrack) {
}

Pose::~Pose() {
}

void Pose::reset(const RigidTransform2D &pose) {
    m_pose = pose;
}

RigidTransform2D Pose::update(
    const double &deltaDistLeftWheel,
    const double &deltaDistRightWheel,
    const double &deltaYawGyro) {
    
    // wheel odometry observation
    double robotDeltaVelWheelObs;
    double robotDeltaYawWheelObs;
    // m_kinematics.forwardKinematics(deltaDistLeftWheel, deltaDistRightWheel,
        // robotDeltaVelWheelObs, robotDeltaYawWheelObs);

    // gyro observation
    double robotDeltaYawGyroObs = deltaYawGyro;

    // update pose
    RigidTransform2D deltaPose =
        RigidTransform2D(Translation2D(0, robotDeltaVelWheelObs),
            Rotation2D::fromDegrees(robotDeltaYawGyroObs));
    m_pose = m_pose.transformBy(deltaPose);

    return m_pose;

    // @TODO: potential improvements...
    // fuse multiple observations from different sensors (IMU, lidar, etc.)
    // trapezoidal integration instead of rectangular integration
    // lateral slip estimation
}
