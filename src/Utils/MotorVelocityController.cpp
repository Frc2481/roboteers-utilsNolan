#include "MotorVelocityController.h"

MotorVelocityController::MotorVelocityController() {
}

MotorVelocityController::MotorVelocityController(
    TalonSRX* talon,
    uint32_t driveMotorID,
    const double &kp,
    const double &ki,
    const double &kd,
    const double &kv,
    const double &ka,
    const double &iZone,
    const double &iErrorLim,
    unsigned ticksPerRev)
    
    : m_pDriveMotor(talon),
    m_kv(kv),
    m_ka(ka),
    m_ticksPerRev(ticksPerRev) {

    m_pDriveMotor->SelectProfileSlot(0, 0);
	m_pDriveMotor->Set(ControlMode::Velocity, 0);
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
	m_pDriveMotor->SetInverted(false);
}

MotorVelocityController::~MotorVelocityController() {
}

void MotorVelocityController::update(const double &refV, const double &refA) {
    refV = refV * m_ticksPerRev / 10.0; // convert to talon native units
    refA = refA * m_ticksPerRev / 10.0 / 100.0; // convert to talon native units
    double feedforwardControl = refV * m_kv + refA * m_ka;
    m_pDriveMotor->Set(ControlMode::Velocity, refV, DemandType::ArbitraryFeedForward, feedforwardControl);
}