#include "MotorPositionController.h"

MotorPositionController::MotorPositionController()
	: m_pDriveMotor(nullptr),
	m_kv(0),
	m_ka(0),
	m_ticksPerRev(0) {
}

MotorPositionController::MotorPositionController(
    TalonSRX* pTalon,
    bool inverted,
    double kp,
    double ki,
    double kd,
    double kv,
    double ka,
    double iZone,
    double iErrorLim,
    unsigned ticksPerRev)
    
    : m_pDriveMotor(pTalon),
    m_kv(kv),
    m_ka(ka),
    m_ticksPerRev(ticksPerRev) {
    
    m_pDriveMotor->SelectProfileSlot(0, 0);
	m_pDriveMotor->Set(ControlMode::Position, 0);
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

MotorPositionController::~MotorPositionController() {
}

void MotorPositionController::update(double refP, double refV, double refA) {
    refV = refV * m_ticksPerRev / 10.0; // convert to talon native units
    refA = refA * m_ticksPerRev / 10.0; // convert to talon native units
    double feedforwardControl = refV * m_kv + refA * m_ka;
    m_pDriveMotor->Set(ControlMode::Position, refP, DemandType::DemandType_ArbitraryFeedForward, feedforwardControl);
}
