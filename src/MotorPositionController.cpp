#include "MotorPositionController.h"

MotorPositionController::MotorPositionController(
		const uint32_t &driveMotorID,
        const double &kp,
		const double &ki,
		const double &kd,
		const double &kv,
		const double &ka,
        const double &iZone,
        const double &iErrorLim) {
    m_driveMotor = new TalonSRX(driveMotorID);
    m_driveMotor->SelectProfileSlot(0, 0);
	m_driveMotor->Set(ControlMode::Position, 0);
	m_driveMotor->Config_kP(0, kp, 0);
	m_driveMotor->Config_kI(0, ki, 0);
	m_driveMotor->Config_kD(0, kd, 0);
    m_driveMotor->Config_IntegralZone(0, iZone, 0);
    m_driveMotor->ConfigMaxIntegralAccumulator (0, iErrorLim, 0);
    m_driveMotor->SetNeutralMode(NeutralMode::Brake);
    m_driveMotor->EnableVoltageCompensation(true);
    m_driveMotor->ConfigVoltageCompSaturation(12.0, 0);
    m_driveMotor->ConfigNeutralDeadband(0.04, 0);
    m_driveMotor->ConfigNominalOutputForward(0.0, 0.0);
	m_driveMotor->ConfigNominalOutputReverse(0.0, 0.0);
	m_driveMotor->ConfigPeakOutputForward(1.0, 0.0);
	m_driveMotor->ConfigPeakOutputReverse(-1.0, 0.0);
	m_driveMotor->SetSensorPhase(true);
	m_driveMotor->SetInverted(false);
    m_kv = kv;
    m_ka = ka;
}

void MotorPositionController::update(const double &refP, const double &refV, const double &refA) {
    double feedforwardControl = refV * m_kv + refA * m_ka;
    m_driveMotor->Set(ControlMode::Position, refP, DemandType::ArbitraryFeedForward, feedforwardControl);
}
