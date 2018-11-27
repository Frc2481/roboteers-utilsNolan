#include "MotorVelocityController.h"

MotorVelocityController::MotorVelocityController() {
}

MotorVelocityController::MotorVelocityController(
    TalonSRX* pTalon,
    bool inverted,
    const double &kp,
    const double &ki,
    const double &kd,
    const double &kv,
    const double &ka,
    const double &iZone,
    const double &iErrorLim,
    unsigned ticksPerRev)
    
    : m_pDriveMotor(pTalon),
    m_kv(kv),
    m_ka(ka),
    m_ticksPerRev(ticksPerRev) {

    m_pDriveMotor->SelectProfileSlot(0, 0);
	m_pDriveMotor->Set(ControlMode::PercentOutput, 0);
	m_pDriveMotor->Config_kP(0, kp, 0);
	m_pDriveMotor->Config_kI(0, ki, 0);
	m_pDriveMotor->Config_kD(0, kd, 0);
    m_pDriveMotor->Config_IntegralZone(0, iZone, 0);
    m_pDriveMotor->ConfigMaxIntegralAccumulator (0, iErrorLim, 0);
    m_pDriveMotor->SetNeutralMode(NeutralMode::Brake);
    m_pDriveMotor->EnableVoltageCompensation(true);
    m_pDriveMotor->ConfigVoltageCompSaturation(12.0, 0);
    m_pDriveMotor->ConfigNeutralDeadband(0.04, 0);
    m_pDriveMotor->ConfigNominalOutputForward(0.0, 0.0);
	m_pDriveMotor->ConfigNominalOutputReverse(0.0, 0.0);
	m_pDriveMotor->ConfigPeakOutputForward(1.0, 0.0);
	m_pDriveMotor->ConfigPeakOutputReverse(-1.0, 0.0);
	m_pDriveMotor->SetSensorPhase(true);
	m_pDriveMotor->SetInverted(inverted);
}

MotorVelocityController::~MotorVelocityController() {
}

void MotorVelocityController::updateClosedLoopControl(double refV, double refA) {
    refV = refV * m_ticksPerRev / 10.0; // convert to talon native units
    refA = refA * m_ticksPerRev / 10.0; // convert to talon native units
    double feedforwardControl = refV * m_kv + refA * m_ka;
    m_pDriveMotor->Set(ControlMode::Velocity, refA, DemandType::DemandType_ArbitraryFeedForward, feedforwardControl);
}

void MotorVelocityController::updateOpenLoopControl(const double &refPercent) {
    m_pDriveMotor->Set(ControlMode::PercentOutput, refPercent);
}
