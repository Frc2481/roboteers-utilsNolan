#ifndef MOTOR_VELOCITY_CONTROLLER_H
#define MOTOR_VELOCITY_CONTROLLER_H

#include "ctre/Phoenix.h"

class MotorVelocityController {
public:
    void MotorVelocityController();
    void MotorVelocityController(
		TalonSRX* talon,
        const double &kp,
		const double &ki,
		const double &kd,
		const double &kv,
		const double &ka,
        const double &iZone,
        const double &iErrorLim,
        unsigned ticksPerRev);
    void ~MotorVelocityController();

    //////////////////////////////////////////////////////////////////////
    // @brief update reference points of closed loop motor controller
    // @param refV - angular velocity reference point (deg/s)
    // @param refA - angular acceleration reference point (deg/s^2)
    //////////////////////////////////////////////////////////////////////
    void updateClosedLoopControl(const double &refV, const double &refA);

    //////////////////////////////////////////////////////////////////////
    // @brief update reference points of open loop motor controller
    // @param refPercent - percent reference point (-1 to 1)
    //////////////////////////////////////////////////////////////////////
    void updateOpenLoopControl(const double &refPercent);

private:
    TalonSRX* m_pDriveMotor;
    double m_kv;
    double m_ka;
    unsigned m_ticksPerRev;
};

#endif // MOTOR_VELOCITY_CONTROLLER_H
