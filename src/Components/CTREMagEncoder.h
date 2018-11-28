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
    double getWheelDistance(double wheelRadius, double gearRatioEncoderToWheel) const;
    double convertRevsToTicks(double revs) const;
    double convertRevsToTickSetpoint(double revs) const;
    double convertAngleToTicks(double angle) const;
    double convertAngleToTickSetpoint(double angle) const;
    double convertWheelDistanceToRevs(double wheelRadius,  double wheelDistance) const;
    double convertWheelDistanceToTicks(double wheelRadius, double wheelDistance) const;
    double convertWheelDistanceToTickSetpoint(double wheelRadius, double wheelDistance) const;
    bool IsConnected() const;

private:
    TalonSRX* m_pTalon;
    int m_encoderTicks;
    int m_encoderTicksZero;
    std::string m_calibrationKey;
};

#endif // CTRE_MAG_ENCODER_H
