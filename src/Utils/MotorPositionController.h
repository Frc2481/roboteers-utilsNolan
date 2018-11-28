#ifndef MOTOR_POSITION_CONTROLLER_H
#define MOTOR_POSITION_CONTROLLER_H

#include "ctre/Phoenix.h"

class MotorPositionController {
public:
    MotorPositionController();
    MotorPositionController(
		TalonSRX* pTalon,
        bool inverted,
        double kp,
		double ki,
		double kd,
		double kv,
		double ka,
        double iZone,
        double iErrorLim,
        unsigned ticksPerRev);
    ~MotorPositionController();

    //////////////////////////////////////////////////////////////////////
    // @brief update reference points of motor controller
    // @param refP - position reference point (ticks)
    // @param refV - angular velocity reference point (deg/s)
    // @param refA - angular acceleration reference point (deg/s^2)
    //////////////////////////////////////////////////////////////////////
    void update(double refP, const double refV, double refA);

private:
    TalonSRX* m_pDriveMotor;
    double m_kv;
    double m_ka;
    unsigned m_ticksPerRev;
};

#endif // MOTOR_POSITION_CONTROLLER_H
