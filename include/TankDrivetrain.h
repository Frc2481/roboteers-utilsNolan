#ifndef TANK_DRIVETRAIN_H
#define TANK_DRIVETRAIN_H

#include "TankDriveKinematics.h"
#include "MotorVelocityController.h"

// +x = robot right
// +y = robot forward
// +yaw = CCW, zero is robot forward

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

class TankDrivetrain {
public:
    void TankDrivetrain();
    void TankDrivetrain(
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
        unsigned updateRate);
    void ~TankDrivetrain();

    void setMaxSpeed(const double &maxSpeed);
    void setMaxAccel(const double &maxAccel);
    void setMaxDeccel(const double &maxDeccel);

    //////////////////////////////////////////////////////////////////////
    // @brief command robot to drive
    // @param robotVel - forward velocity of robot (in/s)
    // @param robotYawRate - yaw rate of robot (deg/s)
    // &param robotAccel - forward acceleration of robot (in/s^2)
    //////////////////////////////////////////////////////////////////////
    void drive(
        const double &robotVel,
        const double &robotYawRate,
        const double &robotAccel,
        const double &time);

private:
    double m_wheelRad; // wheel radius (in)
    double m_wheelTrack; // wheel track (in)
    TankDriveKinematics m_kinematics;
    MotorVelocityController m_leftDriveMotorController;
    MotorVelocityController m_rightDriveMotorController;
    double m_maxSpeed; // max speed (in/s)
    double m_maxAccel; // max accel (in/s^2)
    double m_maxDeccel; // max deccel (in/s^2)
    double m_maxCentripAccel; // max centripetal accel (in/s^2)
    double m_updateRate; // update rate (Hz)
};

#endif // TANK_DRIVETRAIN_H
