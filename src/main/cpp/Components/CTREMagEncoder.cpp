#include "CTREMagEncoder.h"
#include <sstream>
#include <frc/WPILib.h>
#include "../RobotParameters.h"
#include "../Utils/MathConstants.h"
#include "../Utils/NormalizeToRange.h"

CTREMagEncoder::CTREMagEncoder(TalonSRX* pTalon, const std::string &name)
	: m_pTalon(pTalon),
    m_encoderTicks(0),
    m_encoderTicksZero(0) {

    std::stringstream ss;
	ss << "ENCODER_OFFSET_" << name;
	m_calibrationKey = ss.str();
    m_encoderTicksZero = frc::Preferences::GetInstance()->GetDouble(m_calibrationKey);

    m_pTalon->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute, 0, 0);
    m_pTalon->SetStatusFramePeriod(Status_2_Feedback0, 10, 0);
}

CTREMagEncoder::~CTREMagEncoder() {
}

void CTREMagEncoder::update() {
    m_encoderTicks = m_pTalon->GetSelectedSensorPosition(0);
}

void CTREMagEncoder::zero() {
    m_encoderTicksZero = m_pTalon->GetSelectedSensorPosition(0);
    frc::Preferences::GetInstance()->PutDouble(m_calibrationKey, m_encoderTicksZero);
}

void CTREMagEncoder::zeroTalon() {
	m_pTalon->SetSelectedSensorPosition(0, 0, 0);
    zero();
}

int CTREMagEncoder::getTicks() const {
	return m_encoderTicks - m_encoderTicksZero;
}

double CTREMagEncoder::getRevs() const {
    return getTicks() / (double)RobotParameters::k_ctreMagEncoderTicksPerRev;
}

double CTREMagEncoder::getAngle() const {
    return normalizeToRange::normalizeToRange(std::fmod(getRevs(), 1) * 360.0, -180, 180, true);
}

double CTREMagEncoder::getWheelDistance(double wheelRadius, double gearRatioEncoderToWheel) const {
    return getRevs() * gearRatioEncoderToWheel * wheelRadius * 2.0 * MATH_CONSTANTS_PI;
}

double CTREMagEncoder::convertRevsToTicks(double revs) const {
    return revs * (double)RobotParameters::k_ctreMagEncoderTicksPerRev;
}

double CTREMagEncoder::convertRevsToTickSetpoint(double revs) const {
    return convertRevsToTicks(revs) + m_encoderTicksZero;
}

double CTREMagEncoder::convertAngleToTicks(double angle) const {
    return convertRevsToTicks(angle * MATH_CONSTANTS_PI / 180.0);
}

double CTREMagEncoder::convertAngleToTickSetpoint(double angle) const {
    double angleTicks = convertAngleToTicks(angle);
    double currentTicks = getTicks();
    
    double error = angleTicks - currentTicks;
    if(fabs(error) > (double)RobotParameters::k_ctreMagEncoderTicksPerRev / 2.0) {
        if(error > 0) {
            error = error - (double)RobotParameters::k_ctreMagEncoderTicksPerRev;
        }
        else {
            error = error + (double)RobotParameters::k_ctreMagEncoderTicksPerRev;
        }
    }
    
    return currentTicks + error;
}

double CTREMagEncoder::convertWheelDistanceToRevs(double wheelRadius, double wheelDistance) const {
    return wheelDistance / (wheelRadius * 2.0 * MATH_CONSTANTS_PI);
}

double CTREMagEncoder::convertWheelDistanceToTicks(double wheelRadius, double wheelDistance) const {
    return convertRevsToTicks(convertWheelDistanceToRevs(wheelRadius, wheelDistance));
}

double CTREMagEncoder::convertWheelDistanceToTickSetpoint(double wheelRadius, double wheelDistance) const {
    return convertWheelDistanceToTicks(wheelRadius, wheelDistance) + m_encoderTicksZero;
}

bool CTREMagEncoder::IsConnected() const {
	return m_pTalon->GetSensorCollection().GetPulseWidthRiseToRiseUs() > 0;
}
