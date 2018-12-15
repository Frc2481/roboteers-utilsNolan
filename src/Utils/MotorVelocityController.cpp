#include "MotorVelocityController.h"
#include "Utils/Sign.h"
#include "WPILib.h"

MotorVelocityController::MotorVelocityController()
	: m_pDriveMotor(nullptr),
	m_kv(0),
	m_kap(0),
	m_kan(0),
	m_ksf(0),
	m_ticksPerRev(0) {
}

MotorVelocityController::MotorVelocityController(
    TalonSRX* pTalon,
    bool inverted,
    double kp,
    double ki,
    double kd,
    double kv,
    double kap,
	double kan,
	double ksf,
    double iZone,
    double iErrorLim,
    unsigned ticksPerRev)
    
    : m_pDriveMotor(pTalon),
    m_kv(kv),
    m_kap(kap),
	m_kan(kan),
	m_ksf(ksf),
    m_ticksPerRev(ticksPerRev) {

    m_pDriveMotor->SelectProfileSlot(0, 0);
	m_pDriveMotor->Set(ControlMode::PercentOutput, 0);
	m_pDriveMotor->Config_kP(0, kp, 0);
	m_pDriveMotor->Config_kI(0, ki, 0);
	m_pDriveMotor->Config_kD(0, kd, 0);
	m_pDriveMotor->Config_kF(0, 0, 0);
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

void MotorVelocityController::setTicksPerRev(unsigned ticksPerRev) {
	m_ticksPerRev = ticksPerRev;
}

void MotorVelocityController::updateClosedLoopControl(double refV, double refA) {
    refV = refV * m_ticksPerRev / 360.0 / 10.0; // convert to talon native units
    refA = refA * m_ticksPerRev / 360.0 / 10.0; // convert to talon native units

    // use different ka if vel and accel have opposite direction
    double ka = m_kap;
	if((refV > 0) != (refA > 0)) {
		ka = 0; // m_kan;
		refV = 0;
	}

    double feedforwardControl = refV * m_kv + refA * ka + Sign::sign(refV) * m_ksf;

    m_pDriveMotor->Set(ControlMode::Velocity, refV, DemandType::DemandType_ArbitraryFeedForward, feedforwardControl);
}

void MotorVelocityController::updateOpenLoopControl(double refPercent) {
    m_pDriveMotor->Set(ControlMode::PercentOutput, refPercent);
}
