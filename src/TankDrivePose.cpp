#include "TankDrivePose.h"
#include <math.h>

TankDrivePose::TankDrivePose(const Pose2D &pose, const double &wheelTrack)
    : m_pose(pose),
    m_kinematics(wheelTrack),
    m_cornerStiffCoeff(0) {
}

TankDrivePose::~TankDrivePose() {
}

void TankDrivePose::setCornerStiffCoeff(const double &cornerStiffCoeff) {
    m_cornerStiffCoeff = cornerStiffCoeff;
}

void TankDrivePose::reset(const Pose2D &pose) {
    m_pose = pose;
}

Pose2D TankDrivePose::update(
    const double &deltaDistLeftWheel,
    const double &deltaDistRightWheel,
    const double &deltaYawGyro) {
    
    // wheel odometry measurement
    double robotDeltaDistWheelMeas;
    double robotDeltaYawWheelMeas;
    m_kinematics.forwardKinematics(deltaDistLeftWheel, deltaDistRightWheel,
        robotDeltaDistWheelMeas, robotDeltaYawWheelMeas);

    // gyro measurement
    double robotDeltaYawGyroMeas = deltaYawGyro;

    // estimate lateral wheel slip
    double deltaCentripAccel = -robotDeltaDistWheelMeas * deltaYawGyro * M_PI / 180.0;
    double deltaLatWheelSlip = robotDeltaDistWheelMeas
        * -deltaCentripAccel / 386.1 * m_cornerStiffCoeff; // 386.1 in/s^2 = 9.81 m/s^2

    // update pose with measurements
    Translation2D deltaPosRobotFrame = Translation2D(deltaLatWheelSlip, robotDeltaDistWheelMeas);
    Rotation2D newRobotYaw = m_pose.getRotation().rotateBy(Rotation2D::fromDegrees(robotDeltaYawGyroMeas));
    Translation2D deltaPosGlobalFrame = (deltaPosRobotFrame.rotateBy(m_pose.getRotation())
        + deltaPosRobotFrame.rotateBy(newRobotYaw)).scaleBy(0.5); // trapezoidal integration
    Translation2D newRobotPos = m_pose.getTranslation().translateBy(deltaPosGlobalFrame);
    m_pose = Pose2D(newRobotPos, newRobotYaw);

    return m_pose;

    // @TODO: potential improvements...
    // fuse multiple observations from different sensors (IMU, lidar, etc.)
}