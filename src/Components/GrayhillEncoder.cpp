#include "GrayhillEncoder.h"
#include <math.h>
#include "WPILib.h"

GrayhillEncoder::GrayhillEncoder(TalonSRX* talon, const std::string &name),
	: m_talon(talon),
    m_encoderTicks(0),
    m_encoderTicksZero(0),
    m_name(name) {

	m_talon->SetStatusFramePeriod(Status_2_Feedback0, 10, 0);
}

GrayhillEncoder::~GrayhillEncoder() {
}

void GrayhillEncoder::update() {
    m_encoderTicks = m_talon->GetSelectedSensorPosition(0);
    m_encoderTickVel = m_talon->getSelectedSensorVelocity(0);
}

void GrayhillEncoder::zero() {
    m_encoderTicksZero = m_talon->GetSelectedSensorPosition(0);
}

bool GrayhillEncoder::isZeroed() const {
    return m_encoderTicksZero != 0;
}

int GrayhillEncoder::getTicks() const {
	return m_encoderTicks - m_encoderTicksZero;
}

int GrayhillEncoder::getTickVelocity() const {
    return m_encoderTickVel * 10; // convert from talon native units
}

double GrayhillEncoder::getRevs() const {
    return getEncoderTicks() / (double)TICKS_PER_REV;
}

double GrayhillEncoder::getRevVelocity() const {
    return getTickVelocity() / (double)TICKS_PER_REV;
}

double GrayhillEncoder::getAngle() const {
    return (getEncoderRevs() % 1) * 180.0 / M_PI;
}

double GrayhillEncoder::getWheelDistance(const double &wheelRadius, const double &gearRatioEncoderToWheel) const {
    return getEncoderRevs() * gearRatioEncoderToWheel * wheelRadius * 2.0 * M_PI;
}

double GrayhillEncoder::getWheelVelocity(const double &wheelRadius, const double &gearRatioEncoderToWheel) const {
    return getRevVelocity() * gearRatioEncoderToWheel * wheelRadius * 2.0 * M_PI;
}

double GrayhillEncoder::convertRevsToTicks(const double &revs) const {
    return revs * (double)TICKS_PER_REV;
}

double GrayhillEncoder::convertRevsToTickSetpoint(const double &revs) const {
    return convertRevsToTicks(revs) + m_encoderTicksZero;
}

double GrayhillEncoder::convertAngleToTicks(const double &angle) const {
    return convertRevsToTicks(angle * M_PI / 180.0);
}

double GrayhillEncoder::convertAngleToTickSetpoint(const double &angle) const {
    double angleTicks = convertAngleToTicks(angle);
    double currentTicks = getTicks();
    
    double error = angleTicks - currentTicks;
    if(fabs(error) > (double)TICKS_PER_REV / 2.0) {
        if(error > 0) {
            error = error - (double)TICKS_PER_REV;
        }
        else {
            error = error + (double)TICKS_PER_REV;
        }
    }
    
    return currentTicks + error;
}

double GrayhillEncoder::convertWheelDistanceToRevs(const double &wheelDistance) const {
    return wheelDistance / (wheelRadius * 2.0 * M_PI);
}

double GrayhillEncoder::convertWheelDistanceToTicks(const double &wheelDistance) const {
    return convertRevsToTicks(convertWheelDistanceToRevs(wheelDistance));
}

double GrayhillEncoder::convertWheelDistanceToTickSetpoint(const double &wheelDistance) const {
    return convertWheelDistanceToTicks(wheelDistance) + m_encoderTicksZero;
}
