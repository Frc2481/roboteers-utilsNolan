#ifndef GRAYHILL_ENCODER_H
#define GRAYHILL_ENCODER_H

#include <string>
#include "ctre/Phoenix.h"

class GrayhillEncoder {
public:
    GrayhillEncoder(TalonSRX* pTalon, const std::string &name);
    ~GrayhillEncoder();

    void update();
    void zero();
    bool isZeroed() const;
    int getTicks() const;
    int getTickVelocity() const;
    double getRevs() const;
    double getRevVelocity() const;
    double getAngle() const;
    double getWheelDistance(const double &wheelRadius, const double &gearRatioEncoderToWheel) const;
    double getWheelVelocity(const double &wheelRadius, const double &gearRatioEncoderToWheel) const;
    double convertRevsToTicks(const double &revs) const;
    double convertRevsToTickSetpoint(const double &revs) const;
    double convertAngleToTicks(const double &angle) const;
    double convertAngleToTickSetpoint(const double &angle) const;
    double convertWheelDistanceToRevs(const double &wheelRadius,  const double &wheelDistance) const;
    double convertWheelDistanceToTicks(const double &wheelRadius, const double &wheelDistance) const;
    double convertWheelDistanceToTickSetpoint(const double &wheelRadius, const double &wheelDistance) const;

private:
    TalonSRX* m_pTalon;
    int m_encoderTicks;
    int m_encoderTicksZero;
    std::string m_name;
    double m_encoderTickVel;
};

#endif // GRAYHILL_ENCODER_H
