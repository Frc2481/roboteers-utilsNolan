#include "MotorVelocityController.h"
#include "frc/WPILib.h"
#include "../RobotParameters.h"
#include "Sign.h"

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

	frc::SmartDashboard::PutNumber("k_driveMotorControllerKp", RobotParameters::k_driveMotorControllerKp);
	frc::SmartDashboard::PutNumber("k_driveMotorControllerKi", RobotParameters::k_driveMotorControllerKi);
	frc::SmartDashboard::PutNumber("k_driveMotorControllerKd", RobotParameters::k_driveMotorControllerKd);
	frc::SmartDashboard::PutNumber("k_driveMotorControllerKv", RobotParameters::k_driveMotorControllerKv);
	frc::SmartDashboard::PutNumber("k_driveMotorControllerKap", RobotParameters::k_driveMotorControllerKap);
	frc::SmartDashboard::PutNumber("k_driveMotorControllerKan", RobotParameters::k_driveMotorControllerKan);
	frc::SmartDashboard::PutNumber("k_driveMotorControllerKsf", RobotParameters::k_driveMotorControllerKsf);
	frc::SmartDashboard::PutNumber("feedforwardControl", 0);
}

MotorVelocityController::~MotorVelocityController() {
}

void MotorVelocityController::setTicksPerRev(unsigned ticksPerRev) {
	m_ticksPerRev = ticksPerRev;
}

void MotorVelocityController::updateClosedLoopControl(double refV, double refA) {
	double kp = frc::SmartDashboard::GetNumber("k_driveMotorControllerKp", 0);
	double ki = frc::SmartDashboard::GetNumber("k_driveMotorControllerKi", 0);
	double kd = frc::SmartDashboard::GetNumber("k_driveMotorControllerKd", 0);
	m_kv = frc::SmartDashboard::GetNumber("k_driveMotorControllerKv", 0);
	m_kap = frc::SmartDashboard::GetNumber("k_driveMotorControllerKap", 0);
	m_kan = frc::SmartDashboard::GetNumber("k_driveMotorControllerKan", 0);
	m_ksf = frc::SmartDashboard::GetNumber("k_driveMotorControllerKsf", 0);

	m_pDriveMotor->Config_kP(0, kp, 0);
	m_pDriveMotor->Config_kI(0, ki, 0);
	m_pDriveMotor->Config_kD(0, kd, 0);

    refV *= m_ticksPerRev / 360.0 / 10.0; // convert to talon native units
    refA *= m_ticksPerRev / 360.0 / 10.0; // convert to talon native units

    // use different ka if vel and accel have opposite direction
    double ka = m_kap;
	if((refV > 0) != (refA > 0)) {
		ka = m_kan;
	}

    double feedforwardControl = refV * m_kv + refA * ka + Sign::sign(refV) * m_ksf;
    printf("1 = %0.1f\n", refV * m_kv);
    printf("2 = %0.1f\n", refA * ka);
    printf("3 = %0.1f\n", Sign::sign(refV) * m_ksf);
    frc::SmartDashboard::PutNumber("feedforwardControl", feedforwardControl);

    m_pDriveMotor->Set(ControlMode::Velocity, refV, DemandType::DemandType_ArbitraryFeedForward, feedforwardControl);
}

void MotorVelocityController::updateOpenLoopControl(double refPercent) {
    m_pDriveMotor->Set(ControlMode::PercentOutput, refPercent);
}
