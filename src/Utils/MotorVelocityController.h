#ifndef MOTOR_VELOCITY_CONTROLLER_H
#define MOTOR_VELOCITY_CONTROLLER_H

#include "ctre/Phoenix.h"

class MotorVelocityController {
public:
    MotorVelocityController();
    MotorVelocityController(
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
    ~MotorVelocityController();

    void setTicksPerRev(unsigned ticksPerRev);

    //////////////////////////////////////////////////////////////////////
    // @brief update reference points of closed loop motor controller
    // @param refV - angular velocity reference point (deg/s)
    // @param refA - angular acceleration reference point (deg/s^2)
    //////////////////////////////////////////////////////////////////////
    void updateClosedLoopControl(double refV, double refA);

    //////////////////////////////////////////////////////////////////////
    // @brief update reference points of open loop motor controller
    // @param refPercent - percent reference point (-1 to 1)
    //////////////////////////////////////////////////////////////////////
    void updateOpenLoopControl(double refPercent);

private:
    TalonSRX* m_pDriveMotor;
    double m_kv;
    double m_ka;
    unsigned m_ticksPerRev;
};

#endif // MOTOR_VELOCITY_CONTROLLER_H
