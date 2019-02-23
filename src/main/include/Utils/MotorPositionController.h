#ifndef MOTOR_POSITION_CONTROLLER_H
#define MOTOR_POSITION_CONTROLLER_H

#include "ctre/Phoenix.h"
#include "Components/CTREMagEncoder.h"

class MotorPositionController {
public:
    MotorPositionController();
    MotorPositionController(
		TalonSRX* pTalon,
        CTREMagEncoder* pEncoder,
        bool phase,
        bool inverted,
        double kp,
		double ki,
		double kd,
		double kv,
		double kap,
		double kan,
        double ksf,
        double iZone,
        double iErrorLim,
        unsigned ticksPerRev);
    ~MotorPositionController();

    void setMotionMagicAngular(bool isEnabled, double maxVel, double maxAccel, double kf, uint16_t curveStrength);
    void setMotionMagicLinear(bool isEnabled, double maxVel, double maxAccel, double kf, uint16_t curveStrength, double wheelRadius);

    //////////////////////////////////////////////////////////////////////
    // @brief update reference points of motor controller
    // @param refP - position reference point (deg)
    // @param refV - angular velocity reference point (deg/s)
    // @param refA - angular acceleration reference point (deg/s^2)
    //////////////////////////////////////////////////////////////////////
    void updateAngular(double refP, double refV, double refA);

    //////////////////////////////////////////////////////////////////////
    // @brief update reference points of motor controller
    // @param refP - position reference point (in)
    // @param refV - angular velocity reference point (in/s)
    // @param refA - angular acceleration reference point (in/s^2)
    //////////////////////////////////////////////////////////////////////
    void updateLinear(double refP, double refV, double refA, double wheelRadius);

private:
    TalonSRX* m_pDriveMotor;
    CTREMagEncoder* m_pEncoder;
    double m_kv;
    double m_kap;
    double m_kan;
    double m_ksf;
    unsigned m_ticksPerRev;
    bool m_enableMotionMagic;
};

#endif // MOTOR_POSITION_CONTROLLER_H
