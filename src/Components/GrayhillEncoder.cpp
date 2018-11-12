#include "GrayhillEncoder.h"
#include <WPILib.h>
#include "RobotParameters.h"

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
    return (getRevs() % 1) * 180.0 / M_PI;
}

double GrayhillEncoder::getWheelDistance(const double &wheelRadius, const double &gearRatioEncoderToWheel) const {
    return getRevs() * gearRatioEncoderToWheel * wheelRadius * 2.0 * M_PI;
}

double GrayhillEncoder::getWheelVelocity(const double &wheelRadius, const double &gearRatioEncoderToWheel) const {
    return getRevVelocity() * gearRatioEncoderToWheel * wheelRadius * 2.0 * M_PI;
}

double GrayhillEncoder::convertRevsToTicks(const double &revs) const {
    return revs * (double)RobotParameters::k_grayhillEncoderTicksPerRev;
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

double GrayhillEncoder::convertWheelDistanceToRevs(const double &wheelRadius, const double &wheelDistance) const {
    return wheelDistance / (wheelRadius * 2.0 * M_PI);
}

double GrayhillEncoder::convertWheelDistanceToTicks(const double &wheelRadius, const double &wheelDistance) const {
    return convertRevsToTicks(convertWheelDistanceToRevs(wheelRadius, wheelDistance));
}

double GrayhillEncoder::convertWheelDistanceToTickSetpoint(const double &wheelRadius, const double &wheelDistance) const {
    return convertWheelDistanceToTicks(wheelRadius, wheelDistance) + m_encoderTicksZero;
}
