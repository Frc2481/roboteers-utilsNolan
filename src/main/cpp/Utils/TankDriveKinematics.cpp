#include "TankDriveKinematics.h"
#include "MathConstants.h"

TankDriveKinematics::TankDriveKinematics(double wheelTrack)
    : m_wheelTrack(wheelTrack) {
}

TankDriveKinematics::~TankDriveKinematics() {
}

void TankDriveKinematics::forwardKinematics(
	double leftWheelVel,
	double rightWheelVel,
	double &robotVel,
	double &robotYawRate) {
    
    robotVel = (leftWheelVel + rightWheelVel) / 2.0;
    robotYawRate = ((rightWheelVel - leftWheelVel) / m_wheelTrack - 2.0 * robotVel) * 180.0 / MATH_CONSTANTS_PI;
}

void TankDriveKinematics::inverseKinematics(
	double robotVel,
	double robotYawRate,
	double &leftWheelVel,
	double &rightWheelVel) {

	robotYawRate *= MATH_CONSTANTS_PI / 180.0; // convert to rad/s
    leftWheelVel = robotVel - m_wheelTrack / 2.0 * robotYawRate;
    rightWheelVel = robotVel + m_wheelTrack / 2.0 * robotYawRate;
}
