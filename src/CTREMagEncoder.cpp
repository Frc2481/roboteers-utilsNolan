#include "CTREMagEncoder.h"
#include <math.h>
#include <sstream>
#include <WPILib.h>

CTREMagEncoder::CTREMagEncoder(TalonSRX* talon, const std::string &name),
	: m_talon(talon),
    m_encoderTicks(0),
    m_encoderTicksZero(0),
    m_ticksPerRev(ticksPerRev),
    m_name(name) {

    std::stringstream ss;
	ss << "ENCODER_OFFSET_" << name;
	m_calibrationKey = ss.str();
    m_encoderTicksZero = Preferences::GetInstance()->GetDouble(m_calibrationKey);

	m_talon->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute, 0, 0);
	m_talon->SetSensorPhase(true);
	m_talon->SetStatusFramePeriod(Status_2_Feedback0, 10, 0);
}

CTREMagEncoder::~CTREMagEncoder() {

}

void CTREMagEncoder::update() {
    m_encoderTicks = m_talon->GetSelectedSensorPosition(0);
}

void CTREMagEncoder::zero() {
    m_encoderTicksZero = m_talon->GetSelectedSensorPosition(0);
    Preferences::GetInstance()->PutDouble(m_calibrationKey, m_encoderTicksZero);
}

bool CTREMagEncoder::isZeroed() {
    return m_encoderTicksZero != 0;
}

int CTREMagEncoder::getTicks() {
	return m_encoderTicks - m_encoderTicksZero;
}

double CTREMagEncoder::getRevs() {
    return getEncoderTicks() / (double)TICKS_PER_REV;
}

double CTREMagEncoder::getAngle() {
    return (getEncoderRevs() % 1) * 180.0 / M_PI;
}

double CTREMagEncoder::getWheelDistance(const double &wheelRadius, const double &gearRatioEncoderToWheel) {
    return getEncoderRevs() * gearRatioEncoderToWheel * wheelRadius * 2.0 * M_PI;
}

double CTREMagEncoder::convertRevsToTicks(const double &revs) {
    return revs * (double)TICKS_PER_REV;
}

double CTREMagEncoder::convertRevsToTickSetpoint(const double &revs) {
    return convertRevsToTicks(revs) + m_encoderTicksZero;
}

double CTREMagEncoder::convertAngleToTicks(const double &angle) {
    return convertRevsToTicks(angle * M_PI / 180.0);
}

double CTREMagEncoder::convertAngleToTickSetpoint(const double &angle) {
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

double CTREMagEncoder::convertWheelDistanceToRevs(const double &wheelDistance) {
    return wheelDistance / (wheelRadius * 2.0 * M_PI);
}

double CTREMagEncoder::convertWheelDistanceToTicks(const double &wheelDistance) {
    return convertRevsToTicks(convertWheelDistanceToRevs(wheelDistance));
}

double CTREMagEncoder::convertWheelDistanceToTickSetpoint(const double &wheelDistance) {
    return convertWheelDistanceToTicks(wheelDistance) + m_encoderTicksZero;
}

bool CTREMagEncoder::IsConnected() {
	return m_talon->GetSensorCollection().GetPulseWidthRiseToRiseUs() > 0;
}
