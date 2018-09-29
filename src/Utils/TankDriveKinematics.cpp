#include "TankDriveKinematics.h"
#include <math.h>

TankDriveKinematics::TankDriveKinematics(const double &wheelTrack)
    : m_wheelTrack(wheelTrack) {
}

TankDriveKinematics::~TankDriveKinematics() {
}

void TankDriveKinematics::setWheelTrack(const double &wheelTrack) {
    m_wheelTrack = wheelTrack;
}

void TankDriveKinematics::forwardKinematics(
        const double &leftWheelVel,
        const double &rightWheelVel,
        double &robotVel,
        double &robotYawRate) {
    
    robotVel = (leftWheelVel + rightWheelVel) / 2.0;
    robotYawRate = ((rightWheelVel - leftWheelVel) / m_wheelTrack - 2.0 * robotVel) * 180.0 / M_PI;
}

void TankDriveKinematics::inverseKinematics(
        const double &robotVel,
        const double &robotYawRate,
        double &leftWheelVel,
        double &rightWheelVel) {
    
    leftWheelVel = robotVel - m_wheelTrack / 2.0 * robotYawRate * M_PI / 180.0;
    rightWheelVel = robotVel + m_wheelTrack / 2.0 * robotYawRate * M_PI / 180.0;
}
