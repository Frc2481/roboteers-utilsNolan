#include "TankDriveKinematics.h"
#include <math.h>

TankDriveKinematics::TankDriveKinematics(double wheelTrack)
    : m_wheelTrack(wheelTrack) {
}

TankDriveKinematics::~TankDriveKinematics() {
}

void TankDriveKinematics::setWheelTrack(double wheelTrack) {
    m_wheelTrack = wheelTrack;
}

void TankDriveKinematics::forwardKinematics(
        double leftWheelVel,
        double rightWheelVel,
        double &robotVel,
        double &robotYawRate) {
    
    robotVel = (leftWheelVel + rightWheelVel) / 2.0;
    robotYawRate = ((rightWheelVel - leftWheelVel) / m_wheelTrack - 2.0 * robotVel) * 180.0 / M_PI;
}

void TankDriveKinematics::inverseKinematics(
        double robotVel,
        double robotYawRate,
        double &leftWheelVel,
        double &rightWheelVel) {
    
    leftWheelVel = robotVel - m_wheelTrack / 2.0 * robotYawRate * M_PI / 180.0;
    rightWheelVel = robotVel + m_wheelTrack / 2.0 * robotYawRate * M_PI / 180.0;
}
