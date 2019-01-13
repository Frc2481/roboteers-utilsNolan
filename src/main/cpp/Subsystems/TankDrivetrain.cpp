#include "TankDrivetrain.h"
#include "../RobotMap.h"
#include "../RobotParameters.h"
#include "../Commands/TankDrivetrainJoystickDrive.h"
#include "../Utils/Sign.h"
#include "../Utils/MathConstants.h"

TankDrivetrain::TankDrivetrain()
    : Subsystem("TankDrivetrain"),
    m_kinematics(RobotParameters::k_wheelTrack),
    m_tankDrivePose(Pose2D(Translation2D(0, 0), Rotation2D::fromDegrees(0)), RobotParameters::k_wheelTrack, RobotParameters::k_cornerStiffCoeff),
    m_leftWheelDist(0),
    m_rightWheelDist(0),
    m_gyroYaw(0) {
    
    m_pLeftDriveMotor = new TalonSRX(LEFT_DRIVE_MOTOR_ID);
    m_pLeftDriveMotorController = new MotorVelocityController(
        m_pLeftDriveMotor,
		RobotParameters::k_leftDriveMotorInverted,
        RobotParameters::k_driveMotorControllerKp,
        RobotParameters::k_driveMotorControllerKi,
        RobotParameters::k_driveMotorControllerKd,
        RobotParameters::k_driveMotorControllerKv * (0.5 + RobotParameters::k_drivetrainTrimKv),
        RobotParameters::k_driveMotorControllerKap,
		RobotParameters::k_driveMotorControllerKan,
		RobotParameters::k_driveMotorControllerKsf,
        0,
		0,
        RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
    m_pLeftDriveEncoder = new GrayhillEncoder(m_pLeftDriveMotor, "LEFT_DRIVE_MOTOR_ENCODER");
    m_pLeftDriveMotorSlave = new TalonSRX(LEFT_DRIVE_MOTOR_SLAVE_ID);
    m_pLeftDriveMotorSlave->Set(ControlMode::Follower, LEFT_DRIVE_MOTOR_ID);
    m_pLeftDriveMotorSlave->SetInverted(RobotParameters::k_leftDriveMotorInverted);
    
    m_pRightDriveMotor = new TalonSRX(RIGHT_DRIVE_MOTOR_ID);
    m_pRightDriveMotorController = new MotorVelocityController(
        m_pRightDriveMotor,
		!RobotParameters::k_leftDriveMotorInverted,
        RobotParameters::k_driveMotorControllerKp,
        RobotParameters::k_driveMotorControllerKi,
        RobotParameters::k_driveMotorControllerKd,
        RobotParameters::k_driveMotorControllerKv * (0.5 - RobotParameters::k_drivetrainTrimKv),
		RobotParameters::k_driveMotorControllerKap,
		RobotParameters::k_driveMotorControllerKan,
		RobotParameters::k_driveMotorControllerKsf,
		0,
		0,
        RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
    m_pRightDriveEncoder = new GrayhillEncoder(m_pRightDriveMotor, "RIGHT_DRIVE_MOTOR_ENCODER");
    m_pRightDriveMotorSlave = new TalonSRX(RIGHT_DRIVE_MOTOR_SLAVE_ID);
    m_pRightDriveMotorSlave->Set(ControlMode::Follower, RIGHT_DRIVE_MOTOR_ID);
    m_pRightDriveMotorSlave->SetInverted(!RobotParameters::k_leftDriveMotorInverted);

//    m_pShifter = new Solenoid(DRIVE_XMSN_SHIFTER_ID);
//    setShiftState(false);

    m_pChassisIMU = new AHRS(SPI::kMXP);

    resetPose(Pose2D(Translation2D(0, 0), Rotation2D::fromDegrees(0)), PoseDot2D(0, 0, 0));
}

// TankDrivetrain::~TankDrivetrain() {
//     delete m_pLeftDriveMotor;
//     m_pLeftDriveMotor = nullptr;

//     delete m_pRightDriveMotor;
//     m_pRightDriveMotor = nullptr;

//     delete m_pLeftDriveMotorController;
//     m_pLeftDriveMotorController = nullptr;

//     delete m_pRightDriveMotorController;
//     m_pRightDriveMotorController = nullptr;

//     delete m_pLeftDriveEncoder;
//     m_pLeftDriveEncoder = nullptr;

//     delete m_pRightDriveEncoder;
//     m_pRightDriveEncoder = nullptr;

//     delete m_pLeftDriveMotorSlave;
//     m_pLeftDriveMotorSlave = nullptr;

//     delete m_pRightDriveMotorSlave;
//     m_pRightDriveMotorSlave = nullptr;

//     delete m_pShifter;
//     m_pShifter = nullptr;

//     delete m_pChassisIMU;
//     m_pChassisIMU = nullptr;
// }

void TankDrivetrain::InitDefaultCommand() {
	SetDefaultCommand(new TankDrivetrainJoystickDrive());
}

void TankDrivetrain::Periodic() {
	// update encoders
	m_pLeftDriveEncoder->update();
	m_pRightDriveEncoder->update();

    // update shift state
	getShiftState();

	// update pose
    updatePose();
}

void TankDrivetrain::driveOpenLoopControl(double percentLeftDrive, double percentRightDrive) {
    m_pLeftDriveMotorController->updateOpenLoopControl(percentLeftDrive);
    m_pRightDriveMotorController->updateOpenLoopControl(percentRightDrive);
}

void TankDrivetrain::driveClosedLoopControl(
    double robotVel,
    double robotYawRate,
    double robotAccel,
	double robotYawAccel) {

    // convert robot vel to wheel vel
    double leftWheelVel;
	double rightWheelVel;
    m_kinematics.inverseKinematics(robotVel, robotYawRate, leftWheelVel, rightWheelVel);

    // limit wheel vel
	double maxWheelSpeed = fabs(std::max(leftWheelVel, rightWheelVel));
	if(maxWheelSpeed > RobotParameters::k_maxSpeed) {
		leftWheelVel *= RobotParameters::k_maxSpeed / maxWheelSpeed;
		rightWheelVel *= RobotParameters::k_maxSpeed / maxWheelSpeed;
	}

    // convert robot accel to wheel accel
	double leftWheelAccel;
	double rightWheelAccel;
	m_kinematics.inverseKinematics(robotAccel, robotYawAccel, leftWheelAccel, rightWheelAccel);

	// limit wheel accel
	double maxWheelAccel = fabs(std::max(leftWheelAccel, rightWheelAccel));
	if(maxWheelAccel > RobotParameters::k_maxAccel) {
		leftWheelAccel *= RobotParameters::k_maxAccel / maxWheelAccel;
		rightWheelAccel *= RobotParameters::k_maxAccel / maxWheelAccel;
	}

    // convert wheel vel from translational to rotational
    double leftWheelAngVel = leftWheelVel / RobotParameters::k_wheelRad * 180.0 / MATH_CONSTANTS_PI;
    double rightWheelAngVel = rightWheelVel / RobotParameters::k_wheelRad * 180.0 / MATH_CONSTANTS_PI;
    double leftWheelAngAccel = leftWheelAccel / RobotParameters::k_wheelRad * 180.0 / MATH_CONSTANTS_PI;
    double rightWheelAngAccel = rightWheelAccel / RobotParameters::k_wheelRad * 180.0 / MATH_CONSTANTS_PI;

    // update motor vel controller
    m_pLeftDriveMotorController->updateClosedLoopControl(leftWheelAngVel, leftWheelAngAccel);
    m_pRightDriveMotorController->updateClosedLoopControl(rightWheelAngVel, rightWheelAngAccel);
}

void TankDrivetrain::stop() {
    driveOpenLoopControl(0, 0);
}

void TankDrivetrain::setShiftState(bool isHighGear) {
//	m_pShifter->Set(isHighGear);
}

bool TankDrivetrain::getShiftState() {
//	bool isHighGear = m_pShifter->Get();
	bool isHighGear = false;

	// set appropriate motor controller gear ratio
	if(isHighGear) {
		m_pLeftDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioHigh);
		m_pRightDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioHigh);
	}
	else {
		m_pLeftDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
		m_pRightDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
	}

	return isHighGear;
}

Pose2D TankDrivetrain::getPose() {
    return m_tankDrivePose.getPose();
}

PoseDot2D TankDrivetrain::getPoseDot() {
    return m_tankDrivePose.getPoseDot();
}

void TankDrivetrain::updatePose() {
    // read left wheel encoder
    double oldLeftWheelDist = m_leftWheelDist;
    m_leftWheelDist = m_pLeftDriveEncoder->getWheelDistance(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
    SmartDashboard::PutNumber("leftWheelDist", m_leftWheelDist);
    double deltaDistLeftWheel = m_leftWheelDist - oldLeftWheelDist;
    double leftWheelVel = m_pLeftDriveEncoder->getWheelVelocity(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
    SmartDashboard::PutNumber("leftWheelVel", leftWheelVel);

//    // check for wheel slip
//    if(fabs(leftWheelVel) > fabs(m_leftWheelVelCmd * RobotParameters::k_wheelSlipNoiseRatio)) {
//            // account for sample time and measurement noise
//        deltaDistLeftWheel = m_leftWheelVelCmd * 1.0 / (double)RobotParameters::k_updateRate;
//        leftWheelVel = m_leftWheelVelCmd;
//    }

    // read right wheel encoder
    double oldRightWheelDist = m_rightWheelDist;
    m_rightWheelDist = m_pRightDriveEncoder->getWheelDistance(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
    SmartDashboard::PutNumber("rightWheelDist", m_rightWheelDist);
    double deltaDistRightWheel = m_rightWheelDist - oldRightWheelDist;
    double rightWheelVel = m_pRightDriveEncoder->getWheelVelocity(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
    SmartDashboard::PutNumber("rightWheelVel", rightWheelVel);

//    // check for wheel slip
//    if(fabs(rightWheelVel) > fabs(m_rightWheelVelCmd * RobotParameters::k_wheelSlipNoiseRatio)) {
//            // account for sample time and measurement noise
//        deltaDistRightWheel = m_rightWheelVelCmd * 1.0 / (double)RobotParameters::k_updateRate;
//        rightWheelVel = m_rightWheelVelCmd;
//    }

    // read IMU
    double oldGyroYaw = m_gyroYaw;
    m_gyroYaw = -m_pChassisIMU->GetYaw();
    double deltaGyroYaw = m_gyroYaw - oldGyroYaw;
    double gyroYawRate = -m_pChassisIMU->GetRate() * 180.0 / MATH_CONSTANTS_PI;

    // update pose
    m_tankDrivePose.update(deltaDistLeftWheel, deltaDistRightWheel, deltaGyroYaw, leftWheelVel, rightWheelVel, gyroYawRate);

    SmartDashboard::PutNumber("x", m_tankDrivePose.getPose().getTranslation().getX());
    SmartDashboard::PutNumber("y", m_tankDrivePose.getPose().getTranslation().getY());
    SmartDashboard::PutNumber("yaw", m_tankDrivePose.getPose().getRotation().getDegrees());

    SmartDashboard::PutNumber("xVel", m_tankDrivePose.getPoseDot().getXVel());
    SmartDashboard::PutNumber("yVel", m_tankDrivePose.getPoseDot().getYVel());
	SmartDashboard::PutNumber("yawRate", m_tankDrivePose.getPoseDot().getYawRate());
}

void TankDrivetrain::resetPose(const Pose2D &pose, const PoseDot2D &poseDot) {
    m_tankDrivePose.reset(pose, poseDot);
}

void TankDrivetrain::zeroDriveEncoders() {
	m_pLeftDriveEncoder->zero();
	m_pRightDriveEncoder->zero();
}

void TankDrivetrain::zeroGyroYaw() {
	m_pChassisIMU->ZeroYaw();
}
