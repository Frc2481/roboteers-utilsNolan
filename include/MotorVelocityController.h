#ifndef MOTOR_VELOCITY_CONTROLLER_H
#define MOTOR_VELOCITY_CONTROLLER_H

#include "ctre/Phoenix.h"

class MotorVelocityController {
public:
    void MotorVelocityController(
		const uint32_t &driveMotorID,
        const double &kp,
		const double &ki,
		const double &kd,
		const double &kv,
		const double &ka,
        const double &iZone,
        const double &iErrorLim);
    void ~MotorVelocityController();

    void update(const double &refV, const double &refA);

private:
    TalonSRX* m_driveMotor;
    double m_kv;
    double m_ka;
};

#endif // MOTOR_VELOCITY_CONTROLLER_H
