#include "TankDrivetrain.h"

TankDrivetrain::TankDrivetrain(
    const double &wheelTrack,
    const double &wheelRad,
    uint32_t leftDriveMotorID,
    uint32_t rightDriveMotorID,
    const double &kp,
    const double &ki,
    const double &kd,
    const double &kv,
    const double &ka,
    const double &iZone,
    const double &iErrorLim,
    unsigned ticksPerRev,
    unsigned updateRate)

    : m_wheelRad(wheelRad),
    m_wheelTrack(wheelTrack),
    m_kinematics(wheelTrack),
    m_leftDriveMotorController(
        leftDriveMotorID,
        kp,
        ki,
        kd,
        kv,
        ka,
        iZone,
        iErrorLim,
        ticksPerRev),
    m_rightDriveMotorController(
        rightDriveMotorID,
        kp,
        ki,
        kd,
        kv,
        ka,
        iZone,
        iErrorLim,
        ticksPerRev),
        m_maxSpeed(1000),
        m_maxAccel(1000),
        m_maxDeccel(-1000),
        m_maxCentripAccel(1000),
        m_updateRate(updateRate) {
}

TankDrivetrain::~TankDrivetrain() {
}

void TankDrivetrain::setMaxSpeed(const double &maxSpeed) {
    m_maxSpeed = fabs(maxSpeed);
}

void TankDrivetrain::setMaxAccel(const double &maxAccel) {
    m_maxAccel = fabs(maxAccel);
}

void TankDrivetrain::setMaxDeccel(const double &maxDeccel) {
    m_maxDeccel = -fabs(maxDeccel);
}

void TankDrivetrain::setMaxCentripAccel(const double &maxCentripAccel) {
    m_maxCentripAccel = fabs(m_maxCentripAccel);
}

void TankDrivetrain::drive(
    const double &robotVel,
    const double &robotYawRate,
    const double &robotAccel) {

    // get sign of vel
    int velSign = sign(robotVel);
    
    // limit robot vel
    if(robotVel > m_maxSpeed) {
        robotVel = m_maxSpeed;
    }
    else if(robotVel < -m_maxSpeed) {
        robotVel = -m_maxSpeed;
    }

    // limit robot accel
    if(robotAccel > m_maxAccel) {
        robotAccel = m_maxAccel;
    }
    else if(robotAccel < m_maxDeccel) {
        robotAccel = m_maxDeccel;
    }

    double deltaTime = time - m_oldTime;
    double limitVelHigh;
    double limitVelLow;

    double avgAccel = (m_maxAccel - m_maxDeccel) * updateRate / 2.0;
    if(fabs(robotVel) < avgAccel) { // account for behavior around zero
        limitVelHigh = avgAccel;
        limitVelLow = -avgAccel;
    }
    if(velSign > 0) {
        limitVelHigh = m_oldVel + m_maxAccel / (double)updateRate;
        limitVelLow = m_oldVel + m_maxDeccel / (double)updateRate;
    }
    else if(velSign < 0) {
        limitVelHigh = m_oldVel - m_maxAccel / (double)updateRate;
        limitVelLow = m_oldVel - m_maxDeccel / (double)updateRate;
    }

    if(robotVel > limitVelHigh) {
        robotVel = limitVelHigh;
    }
    else if(robotVel < limitVelLow) {
        robotVel = limitVelLow;
    }

    // limit centrip accel
    if(fabs(robotVel * robotYawRate * 180.0 / M_PI) > m_maxCentripAccel) {
        robotVel = velSign * m_maxCentripAccel / fabs(robotYawRate * 180.0 / M_PI);
    }

    // convert robot vel to wheel vel
    double leftWheelVel;
    double rightWheelVel;
    m_kinematics.inverseKinematics(robotVel, robotYawRate, leftWheelVel, rightWheelVel);

    // limit wheel vel
    if(leftWheelVel > m_maxSpeed) {
        leftWheelVel = m_maxSpeed;
        rightWheelVel = leftWheelVel - m_wheelTrack * robotYawRate * 180.0 / M_PI;
    }
    else if (leftWheelVel < - m_maxSpeed) {
        m_maxSpeed = -m_maxSpeed;
        rightWheelVel = leftWheelVel - m_wheelTrack * robotYawRate * 180.0 / M_PI;
    }
    if(rightWheelVel > m_maxSpeed) {
        rightWheelVel = m_maxSpeed;
        leftWheelVel = rightWheelVel + m_wheelTrack * robotYawRate * 180.0 / M_PI;
    }
    else if (rightWheelVel < - m_maxSpeed) {
        rightWheelVel = -m_maxSpeed;
        rightWheelVel = rightWheelVel + m_wheelTrack * robotYawRate * 180.0 / M_PI;
    }

    // convert wheel vel from translational to rotational
    double leftWheelAngVel = leftWheelVel / m_wheelRad * 180.0 / M_PI;
    double righttWheelAngVel = rightWheelVel / m_wheelRad * 180.0 / M_PI;
    double leftWheelAngAccel = robotAccel / m_wheelRad * 180.0 / M_PI;
    double rightWheelAngAccel = robotAccel / m_wheelRad * 180.0 / M_PI;
    
    // update motor vel controller
    m_leftDriveMotorController.update(leftWheelAngVel, leftWheelAngAccel);
    m_rightDriveMotorController.update(righttWheelAngVel, rightWheelAngAccel);
}