#include "SwerveDrivePose.h"
#include "MathConstants.h"

SwerveDrivePose::SwerveDrivePose(
    const Pose2D &pose,
    double wheelTrack,
	double wheelBase,
    double cornerStiffCoeff)

    : m_pose(pose),
    m_poseDot(0, 0, 0),
    m_kinematics(
		Translation2D(wheelTrack / 2.0, wheelBase / 2.0),
		Translation2D(wheelTrack / 2.0, -wheelBase / 2.0),
		Translation2D(-wheelTrack / 2.0, -wheelBase / 2.0),
		Translation2D(-wheelTrack / 2.0, wheelBase / 2.0)),
    m_cornerStiffCoeff(cornerStiffCoeff) {
}

SwerveDrivePose::~SwerveDrivePose() {
}

void SwerveDrivePose::reset(const Pose2D &pose, const PoseDot2D &poseDot) {
    m_pose = pose;
    m_poseDot = poseDot;
}

Pose2D SwerveDrivePose::getPose() {
    return m_pose;
}

PoseDot2D SwerveDrivePose::getPoseDot() {
    return m_poseDot;
}

void SwerveDrivePose::update(
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
	double yawRateGyro) {

    // wheel odometry measurement
    Translation2D robotDeltaDistWheelMeas;
    double robotDeltaYawWheelMeas;
    m_kinematics.forwardKinematics(
		Translation2D(0, deltaDistFRWheel).rotateBy(frWheelYaw),
		Translation2D(0, deltaDistBRWheel).rotateBy(brWheelYaw),
		Translation2D(0, deltaDistBLWheel).rotateBy(blWheelYaw),
		Translation2D(0, deltaDistFLWheel).rotateBy(flWheelYaw),
		robotDeltaDistWheelMeas,
		robotDeltaYawWheelMeas);

    // gyro measurement
    double robotDeltaYawGyroMeas = deltaYawGyro;

    // estimate lateral wheel slip
    Translation2D deltaCentripAccel = (robotDeltaDistWheelMeas.scaleBy(
        -deltaYawGyro * MATH_CONSTANTS_PI / 180.0)).rotateBy(
        Rotation2D::fromDegrees(90)); 
    Translation2D deltaLatWheelSlip = deltaCentripAccel.scaleBy(
        -robotDeltaDistWheelMeas.norm() / 386.1 * m_cornerStiffCoeff); // 386.1 in/s^2 = 9.81 m/s^2

    // update pose with measurements
    Translation2D deltaPosRobotFrame = robotDeltaDistWheelMeas + deltaLatWheelSlip;
    Rotation2D newRobotYaw = m_pose.getRotation().rotateBy(Rotation2D::fromDegrees(robotDeltaYawGyroMeas));
    Translation2D deltaPosGlobalFrame = (deltaPosRobotFrame.rotateBy(m_pose.getRotation())
        + deltaPosRobotFrame.rotateBy(newRobotYaw)).scaleBy(0.5); // trapezoidal integration
    Translation2D newRobotPos = m_pose.getTranslation().translateBy(deltaPosGlobalFrame);
    m_pose = Pose2D(newRobotPos, newRobotYaw);

    // update pose dot with measurements
    Translation2D robotVelWheelMeas;
    double robotYawRateWheelMeas;
    m_kinematics.forwardKinematics(
		Translation2D(0, velFRWheel).rotateBy(frWheelYaw),
		Translation2D(0, velBRWheel).rotateBy(brWheelYaw),
		Translation2D(0, velBLWheel).rotateBy(blWheelYaw),
		Translation2D(0, velFLWheel).rotateBy(flWheelYaw),
		robotVelWheelMeas,
		robotYawRateWheelMeas);
    m_poseDot = PoseDot2D(robotVelWheelMeas.getX(), robotVelWheelMeas.getY(), yawRateGyro);

    // @TODO: potential improvements...
    // fuse multiple observations from different sensors (IMU, lidar, etc.)
}
