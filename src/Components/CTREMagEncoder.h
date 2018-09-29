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
    void zeroTalon();
    int getTicks() const;
    double getRevs() const;
    double getAngle() const;
    double getWheelDistance(const double &wheelRadius, const double &gearRatioEncoderToWheel) const;
    double convertRevsToTicks(const double &revs) const;
    double convertRevsToTickSetpoint(const double &revs) const;
    double convertAngleToTicks(const double &angle) const;
    double convertAngleToTickSetpoint(const double &angle) const;
    double convertWheelDistanceToRevs(const double &wheelDistance) const;
    double convertWheelDistanceToTicks(const double &wheelDistance) const;
    double convertWheelDistanceToTickSetpoint(const double &wheelDistance) const;
}

private:
    TalonSRX* m_talon;
    int m_encoderTicks;
    int m_encoderTicksZero;
    std::string m_name;
};

#endif // CTRE_MAG_ENCODER_H
