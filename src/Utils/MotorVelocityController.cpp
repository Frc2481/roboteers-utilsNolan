#include "MotorVelocityController.h"
#include "Utils/Sign.h"

MotorVelocityController::MotorVelocityController()
	: m_pDriveMotor(nullptr),
	m_kv(0),
	m_ka(0),
	m_ticksPerRev(0) {
}

MotorVelocityController::MotorVelocityController(
    TalonSRX* pTalon,
    bool inverted,
    double kp,
    double ki,
    double kd,
    double kv,
    double ka,
	double ksf,
	double kvf,
    double iZone,
    double iErrorLim,
    unsigned ticksPerRev)
    
    : m_pDriveMotor(pTalon),
    m_kv(kv),
    m_ka(ka),
	m_ksf(ksf),
	m_kvf(kvf),
    m_ticksPerRev(ticksPerRev) {

    m_pDriveMotor->SelectProfileSlot(0, 0);
	m_pDriveMotor->Set(ControlMode::PercentOutput, 0);
	m_pDriveMotor->Config_kP(0, kp, 0);
	m_pDriveMotor->Config_kI(0, ki, 0);
	m_pDriveMotor->Config_kD(0, kd, 0);
	m_pDriveMotor->Config_kF(0, 0, 0);
    m_pDriveMotor->Config_IntegralZone(0, iZone, 0);
    m_pDriveMotor->ConfigMaxIntegralAccumulator (0, iErrorLim, 0);
    m_pDriveMotor->SetNeutralMode(NeutralMode::Coast);
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

void MotorVelocityController::setTicksPerRev(unsigned ticksPerRev) {
	m_ticksPerRev = ticksPerRev;
}

void MotorVelocityController::updateClosedLoopControl(double refV, double refA, double measV) {
    refV = refV * m_ticksPerRev / 360.0 / 10.0; // convert to talon native units
    refA = refA * m_ticksPerRev / 360.0 / 10.0; // convert to talon native units
    measV = measV  * m_ticksPerRev / 360.0 / 10.0; // convert to talon native units
    double feedforwardControl = refV * m_kv + refA * m_ka + Sign::sign(refV) * m_ksf + m_kvf * measV;
    m_pDriveMotor->Set(ControlMode::Velocity, 0, DemandType::DemandType_ArbitraryFeedForward, 500);
}

void MotorVelocityController::updateOpenLoopControl(double refPercent) {
    m_pDriveMotor->Set(ControlMode::PercentOutput, refPercent);
}
