#ifndef CTRE_MAG_ENCODER_H
#define CTRE_MAG_ENCODER_H

#include <string>
#include "ctre/Phoenix.h"

#define TICKS_PER_REV 4096

class CTREMagEncoder {
public:
    CTREMagEncoder(TalonSRX* talon, const std::string &name);
    ~CTREMagEncoder();

    void update();
    void zero();
    int getTicks();
    bool isZeroed()
    double getRevs();
    double getAngle();
    double getWheelDistance(const double &wheelRadius, const double &gearRatioEncoderToWheel);
    double convertRevsToTicks(const double &revs);
    double convertRevsToTickSetpoint(const double &revs);
    double convertAngleToTicks(const double &angle);
    double convertAngleToTickSetpoint(const double &angle);
    double convertWheelDistanceToRevs(const double &wheelDistance);
    double convertWheelDistanceToTicks(const double &wheelDistance);
    double convertWheelDistanceToTickSetpoint(const double &wheelDistance);

private:
    TalonSRX* m_talon;
    int m_encoderTicks;
    int m_encoderTicksZero;
    std::string m_name;
};


#endif // CTRE_MAG_ENCODER_H