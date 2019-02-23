#include "Utils/TankDrivePose.h"
#include "Utils/MathConstants.h"

TankDrivePose::TankDrivePose(
    const Pose2D &pose,
    double wheelTrack,
    double cornerStiffCoeff)
    
    : m_pose(pose),
    m_poseDot(0, 0, 0),
    m_kinematics(wheelTrack),
    m_cornerStiffCoeff(cornerStiffCoeff) {
}

TankDrivePose::~TankDrivePose() {
}

void TankDrivePose::reset(const Pose2D &pose, const PoseDot2D &poseDot) {
    m_pose = pose;
    m_poseDot = poseDot;
}

Pose2D TankDrivePose::getPose() {
    return m_pose;
}

PoseDot2D TankDrivePose::getPoseDot() {
    return m_poseDot;
}

void TankDrivePose::update(
    double deltaDistLeftWheel,
    double deltaDistRightWheel,
    double deltaYawGyro,
    double velLeftWheel,
    double velRightWheel,
    double yawRateGyro) {
    
    // wheel odometry measurement
    double robotDeltaDistWheelMeas;
    double robotDeltaYawWheelMeas;
    m_kinematics.forwardKinematics(deltaDistLeftWheel, deltaDistRightWheel,
        robotDeltaDistWheelMeas, robotDeltaYawWheelMeas);

    // gyro measurement
    double robotDeltaYawGyroMeas = deltaYawGyro;

    // estimate lateral wheel slip
    double deltaCentripAccel = -robotDeltaDistWheelMeas * deltaYawGyro * MATH_CONSTANTS_PI / 180.0;
    double deltaLatWheelSlip = robotDeltaDistWheelMeas
        * -deltaCentripAccel / 386.1 * m_cornerStiffCoeff; // 386.1 in/s^2 = 9.81 m/s^2

    // update pose with measurements
    Translation2D deltaPosRobotFrame = Translation2D(deltaLatWheelSlip, robotDeltaDistWheelMeas);
    Rotation2D newRobotYaw = m_pose.getRotation().rotateBy(Rotation2D::fromDegrees(robotDeltaYawGyroMeas));
    Translation2D deltaPosGlobalFrame = (deltaPosRobotFrame.rotateBy(m_pose.getRotation())
        + deltaPosRobotFrame.rotateBy(newRobotYaw)).scaleBy(0.5); // trapezoidal integration
    Translation2D newRobotPos = m_pose.getTranslation().translateBy(deltaPosGlobalFrame);
    m_pose = Pose2D(newRobotPos, newRobotYaw);

    // update pose dot with measurements
    double robotVelWheelMeas;
    double robotYawRateWheelMeas;
    m_kinematics.forwardKinematics(velLeftWheel, velRightWheel,
        robotVelWheelMeas, robotYawRateWheelMeas);
    m_poseDot = PoseDot2D(0, robotVelWheelMeas, yawRateGyro);

    // @TODO: potential improvements...
    // fuse multiple observations from different sensors (IMU, lidar, etc.)
}
