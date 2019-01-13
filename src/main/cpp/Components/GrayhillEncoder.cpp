#include "GrayhillEncoder.h"
#include <frc/WPILib.h>
#include "../RobotParameters.h"
#include "../Utils/MathConstants.h"
#include "../Utils/NormalizeToRange.h"

GrayhillEncoder::GrayhillEncoder(TalonSRX* pTalon, const std::string &name)
	: m_pTalon(pTalon),
    m_encoderTicks(0),
    m_encoderTicksZero(0),
	m_encoderTickVel(0) {

	m_pTalon->SetStatusFramePeriod(Status_2_Feedback0, 10, 0);
}

GrayhillEncoder::~GrayhillEncoder() {
}

void GrayhillEncoder::update() {
    m_encoderTicks = m_pTalon->GetSelectedSensorPosition(0);
    m_encoderTickVel = m_pTalon->GetSelectedSensorVelocity(0);
}

void GrayhillEncoder::zero() {
    m_encoderTicksZero = m_pTalon->GetSelectedSensorPosition(0);
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
    return getTicks() / (double)RobotParameters::k_grayhillEncoderTicksPerRev;
}

double GrayhillEncoder::getRevVelocity() const {
    return getTickVelocity() / (double)RobotParameters::k_grayhillEncoderTicksPerRev;
}

double GrayhillEncoder::getAngle() const {
    return normalizeToRange::normalizeToRange(std::fmod(getRevs(), 1) * 360.0, -180, 180, true);
}

double GrayhillEncoder::getWheelDistance(double wheelRadius, double gearRatioEncoderToWheel) const {
    return getRevs() * gearRatioEncoderToWheel * wheelRadius * 2.0 * MATH_CONSTANTS_PI;
}

double GrayhillEncoder::getWheelVelocity(double wheelRadius, double gearRatioEncoderToWheel) const {
    return getRevVelocity() * gearRatioEncoderToWheel * wheelRadius * 2.0 * MATH_CONSTANTS_PI;
}

double GrayhillEncoder::convertRevsToTicks(double revs) const {
    return revs * (double)RobotParameters::k_grayhillEncoderTicksPerRev;
}

double GrayhillEncoder::convertRevsToTickSetpoint(double revs) const {
    return convertRevsToTicks(revs) + m_encoderTicksZero;
}

double GrayhillEncoder::convertAngleToTicks(double angle) const {
    return convertRevsToTicks(angle * MATH_CONSTANTS_PI / 180.0);
}

double GrayhillEncoder::convertAngleToTickSetpoint(double angle) const {
    double angleTicks = convertAngleToTicks(angle);
    double currentTicks = getTicks();
    
    double error = angleTicks - currentTicks;
    if(fabs(error) > (double)RobotParameters::k_grayhillEncoderTicksPerRev / 2.0) {
        if(error > 0) {
            error = error - (double)RobotParameters::k_grayhillEncoderTicksPerRev;
        }
        else {
            error = error + (double)RobotParameters::k_grayhillEncoderTicksPerRev;
        }
    }
    
    return currentTicks + error;
}

double GrayhillEncoder::convertWheelDistanceToRevs(double wheelRadius, double wheelDistance) const {
    return wheelDistance / (wheelRadius * 2.0 * MATH_CONSTANTS_PI);
}

double GrayhillEncoder::convertWheelDistanceToTicks(double wheelRadius, double wheelDistance) const {
    return convertRevsToTicks(convertWheelDistanceToRevs(wheelRadius, wheelDistance));
}

double GrayhillEncoder::convertWheelDistanceToTickSetpoint(double wheelRadius, double wheelDistance) const {
    return convertWheelDistanceToTicks(wheelRadius, wheelDistance) + m_encoderTicksZero;
}
