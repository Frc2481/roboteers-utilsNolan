#include "MotorPositionController.h"
#include "Sign.h"

MotorPositionController::MotorPositionController()
	: m_pDriveMotor(nullptr),
	m_kv(0),
	m_kap(0),
	m_kan(0),
	m_ticksPerRev(0) {
}

MotorPositionController::MotorPositionController(
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
	refP *= m_ticksPerRev / 360.0;
    refV *= m_ticksPerRev / 360.0 / 10.0; // convert to talon native units
    refA *= m_ticksPerRev / 360.0 / 10.0; // convert to talon native units

    // use different ka if vel and accel have opposite direction
	double ka = m_kap;
	if((refV > 0) != (refA > 0)) {
		ka = m_kan;
	}

    double feedforwardControl = refV * m_kv + refA * ka + Sign::sign(refV) * m_ksf;
    m_pDriveMotor->Set(ControlMode::Position, refP, DemandType::DemandType_ArbitraryFeedForward, feedforwardControl);
}
