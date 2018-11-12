#include "CTREMagEncoder.h"
#include <WPILib.h>
#include <sstream>
#include "RobotParameters.h"

CTREMagEncoder::CTREMagEncoder(TalonSRX* pTalon, const std::string &name)
	: m_pTalon(pTalon),
    m_encoderTicks(0),
    m_encoderTicksZero(0) {

    std::stringstream ss;
	ss << "ENCODER_OFFSET_" << name;
	m_calibrationKey = ss.str();
    m_encoderTicksZero = Preferences::GetInstance()->GetDouble(m_calibrationKey);

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
    Preferences::GetInstance()->PutDouble(m_calibrationKey, m_encoderTicksZero);
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
    return (getRevs() % 1) * 180.0 / M_PI;
}

double CTREMagEncoder::getWheelDistance(const double &wheelRadius, const double &gearRatioEncoderToWheel) const {
    return getRevs() * gearRatioEncoderToWheel * wheelRadius * 2.0 * M_PI;
}

double CTREMagEncoder::convertRevsToTicks(const double &revs) const {
    return revs * (double)RobotParameters::k_ctreMagEncoderTicksPerRev;
}

double CTREMagEncoder::convertRevsToTickSetpoint(const double &revs) const {
    return convertRevsToTicks(revs) + m_encoderTicksZero;
}

double CTREMagEncoder::convertAngleToTicks(const double &angle) const {
    return convertRevsToTicks(angle * M_PI / 180.0);
}

double CTREMagEncoder::convertAngleToTickSetpoint(const double &angle) const {
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

double CTREMagEncoder::convertWheelDistanceToRevs(const double &wheelRadius, const double &wheelDistance) const {
    return wheelDistance / (wheelRadius * 2.0 * M_PI);
}

double CTREMagEncoder::convertWheelDistanceToTicks(const double &wheelRadius, const double &wheelDistance) const {
    return convertRevsToTicks(convertWheelDistanceToRevs(wheelRadius, wheelDistance));
}

double CTREMagEncoder::convertWheelDistanceToTickSetpoint(const double &wheelRadius, const double &wheelDistance) const {
    return convertWheelDistanceToTicks(wheelRadius, wheelDistance) + m_encoderTicksZero;
}

bool CTREMagEncoder::IsConnected() const {
	return m_pTalon->GetSensorCollection().GetPulseWidthRiseToRiseUs() > 0;
}
