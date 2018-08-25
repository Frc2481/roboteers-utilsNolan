#ifndef MOTOR_POSITION_CONTROLLER_H
#define MOTOR_POSITION_CONTROLLER_H

#include "ctre/Phoenix.h"

class MotorPositionController {
public:
    void MotorPositionController(
		const uint32_t &driveMotorID,
        const double &kp,
		const double &ki,
		const double &kd,
		const double &kv,
		const double &ka,
        const double &iZone,
        const double &iErrorLim);
    void ~MotorPositionController();

    void update(const double &refP, const double &refV, const double &refA);

private:
    TalonSRX* m_driveMotor;
    double m_kv;
    double m_ka;
};

#endif // MOTOR_POSITION_CONTROLLER_H
