#ifndef MOTOR_POSITION_CONTROLLER_H
#define MOTOR_POSITION_CONTROLLER_H

#include "ctre/Phoenix.h"

class MotorPositionController {
public:
    void MotorPositionController();
    void MotorPositionController(
		TalonSRX* pTalon,
        bool inverted,
        const double &kp,
		const double &ki,
		const double &kd,
		const double &kv,
		const double &ka,
        const double &iZone,
        const double &iErrorLim,
        unsigned ticksPerRev);
    void ~MotorPositionController();

    //////////////////////////////////////////////////////////////////////
    // @brief update reference points of motor controller
    // @param refP - position reference point (ticks)
    // @param refV - angular velocity reference point (deg/s)
    // @param refA - angular acceleration reference point (deg/s^2)
    //////////////////////////////////////////////////////////////////////
    void update(const double &refP, const double &refV, const double &refA);

private:
    TalonSRX* m_pDriveMotor;
    double m_kv;
    double m_ka;
    unsigned m_ticksPerRev;
};

#endif // MOTOR_POSITION_CONTROLLER_H
