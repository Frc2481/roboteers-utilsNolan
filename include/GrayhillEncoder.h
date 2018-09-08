#ifndef GRAYHILL_ENCODER_H
#define GRAYHILL_ENCODER_H

#include <string>
#include "ctre/Phoenix.h"

#define TICKS_PER_REV 512

class GrayhillEncoder {
public:
    GrayhillEncoder(TalonSRX* talon, const std::string &name);
    ~GrayhillEncoder();

    void update();
    void zero();
    bool isZeroed() const;
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

private:
    TalonSRX* m_talon;
    int m_encoderTicks;
    int m_encoderTicksZero;
    std::string m_name;
};

#endif // GRAYHILL_ENCODER_H
