#include "Utils/MotorPositionController.h"
#include "Utils/Sign.h"

MotorPositionController::MotorPositionController()
	: m_pDriveMotor(nullptr),
	m_kv(0),
	m_kap(0),
	m_kan(0),
	m_ticksPerRev(0),
    m_enableMotionMagic(false) {
}

MotorPositionController::MotorPositionController(
	TalonSRX* pTalon,
    CTREMagEncoder* pEncoder,
    bool phase,
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
    m_pEncoder(pEncoder),
	m_kv(kv),
	m_kap(kap),
	m_kan(kan),
    m_ksf(ksf),
    m_ticksPerRev(ticksPerRev),
    m_enableMotionMagic(false) {
    
    m_pDriveMotor->ConfigFactoryDefault();
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
	m_pDriveMotor->SetSensorPhase(phase);
	m_pDriveMotor->SetInverted(inverted);
}

MotorPositionController::~MotorPositionController() {
}

void MotorPositionController::setMotionMagicAngular(
    bool isEnabled,
    double maxVel,
    double maxAccel,
    double kf,
    uint16_t curveStrength) {
    
    m_enableMotionMagic = isEnabled;
    m_pDriveMotor->SetStatusFramePeriod(Status_10_MotionMagic, 10, 0);

    maxVel = m_pEncoder->convertAngleToTicks(maxVel) / 10.0; // convert to talon native units
    m_pDriveMotor->ConfigMotionCruiseVelocity(maxVel, 0);
    m_pDriveMotor->Config_kF(0, kf, 0);

    maxAccel = m_pEncoder->convertAngleToTicks(maxAccel) / 10.0; // convert to talon native units
	m_pDriveMotor->ConfigMotionAcceleration(maxAccel, 0);

    m_pDriveMotor->ConfigMotionSCurveStrength(curveStrength);
}

void MotorPositionController::setMotionMagicLinear(
    bool isEnabled,
    double maxVel,
    double maxAccel,
    double kf,
    uint16_t curveStrength,
    double wheelRadius) {
    
    m_enableMotionMagic = isEnabled;
    m_pDriveMotor->SetStatusFramePeriod(Status_10_MotionMagic, 10, 0);

    maxVel = m_pEncoder->convertWheelDistanceToTicks(wheelRadius, maxVel) / 10.0; // convert to talon native units
    m_pDriveMotor->ConfigMotionCruiseVelocity(maxVel, 0);
    m_pDriveMotor->Config_kF(0, kf, 0);

    maxAccel = m_pEncoder->convertWheelDistanceToTicks(wheelRadius, maxAccel) / 10.0; // convert to talon native units
	m_pDriveMotor->ConfigMotionAcceleration(maxAccel, 0);

    m_pDriveMotor->ConfigMotionSCurveStrength(curveStrength);
}

void MotorPositionController::updateAngular(double refP, double refV, double refA) {
    refP = m_pEncoder->convertAngleToTickSetpoint(refP);
    refV = m_pEncoder->convertAngleToTicks(refV) / 10.0; // convert to talon native units
    refA = m_pEncoder->convertAngleToTicks(refA) / 10.0; // convert to talon native units

    // use different ka if vel and accel have opposite direction
	double ka = m_kap;
	if((refV > 0) != (refA > 0)) {
		ka = m_kan;
	}

    double feedforwardControl = refV * m_kv + refA * ka + Sign::Sign(refV) * m_ksf;

    if(!m_enableMotionMagic) {
        m_pDriveMotor->Set(ControlMode::Position, refP, DemandType::DemandType_ArbitraryFeedForward, feedforwardControl);
    }
    else {
        m_pDriveMotor->Set(ControlMode::MotionMagic, refP, DemandType::DemandType_ArbitraryFeedForward, feedforwardControl);
    }
}

void MotorPositionController::updateLinear(double refP, double refV, double refA, double wheelRadius) {
    refP = m_pEncoder->convertWheelDistanceToTickSetpoint(wheelRadius, refP);
    refV = m_pEncoder->convertWheelDistanceToTicks(wheelRadius, refV) / 10.0; // convert to talon native units
    refA = m_pEncoder->convertWheelDistanceToTicks(wheelRadius, refA) / 10.0; // convert to talon native units

    // use different ka if vel and accel have opposite direction
	double ka = m_kap;
	if((refV > 0) != (refA > 0)) {
		ka = m_kan;
	}

    double feedforwardControl = refV * m_kv + refA * ka + Sign::Sign(refV) * m_ksf;
    
    if(!m_enableMotionMagic) {
        m_pDriveMotor->Set(ControlMode::Position, refP, DemandType::DemandType_ArbitraryFeedForward, feedforwardControl);
    }
    else {
        m_pDriveMotor->Set(ControlMode::MotionMagic, refP, DemandType::DemandType_ArbitraryFeedForward, feedforwardControl);
    }
}
