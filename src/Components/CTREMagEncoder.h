#ifndef CTRE_MAG_ENCODER_H
#define CTRE_MAG_ENCODER_H

#include <string>
#include "ctre/Phoenix.h"

class CTREMagEncoder {
public:
    CTREMagEncoder(TalonSRX* pTalon, const std::string &name);
    ~CTREMagEncoder();

    void update();
    void zero();
    void zeroTalon();
    int getTicks() const;
    double getRevs() const;
    double getAngle() const;
    double getWheelDistance(const double &wheelRadius, const double &gearRatioEncoderToWheel) const;
    double convertRevsToTicks(const double &revs) const;
    double convertRevsToTickSetpoint(const double &revs) const;
    double convertAngleToTicks(const double &angle) const;
    double convertAngleToTickSetpoint(const double &angle) const;
    double convertWheelDistanceToRevs(const double &wheelRadius,  const double &wheelDistance) const;
    double convertWheelDistanceToTicks(const double &wheelRadius, const double &wheelDistance) const;
    double convertWheelDistanceToTickSetpoint(const double &wheelRadius, const double &wheelDistance) const;
    bool IsConnected() const;

private:
    TalonSRX* m_pTalon;
    int m_encoderTicks;
    int m_encoderTicksZero;
    std::string m_calibrationKey;
};

#endif // CTRE_MAG_ENCODER_H
