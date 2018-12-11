#include "TankDrivetrain.h"
#include "RobotMap.h"
#include "RobotParameters.h"
#include "Commands/TankDrivetrainJoystickDrive.h"
#include "Utils/Sign.h"

TankDrivetrain::TankDrivetrain()
    : Subsystem("TankDrivetrain"),
    m_kinematics(RobotParameters::k_wheelTrack),
    m_tankDrivePose(Pose2D(Translation2D(0, 0), Rotation2D::fromDegrees(0)), RobotParameters::k_wheelTrack, RobotParameters::k_cornerStiffCoeff),
    m_leftWheelDist(0),
    m_rightWheelDist(0),
    m_leftWheelVelCmd(0),
    m_rightWheelVelCmd(0),
    m_gyroYaw(0) {
    
    m_pLeftDriveMotor = new TalonSRX(LEFT_DRIVE_MOTOR_ID);
    m_pLeftDriveMotorController = new MotorVelocityController(
        m_pLeftDriveMotor,
		RobotParameters::k_leftDriveMotorInverted,
        RobotParameters::k_driveMotorControllerKp,
        RobotParameters::k_driveMotorControllerKi,
        RobotParameters::k_driveMotorControllerKd,
        RobotParameters::k_driveMotorControllerKv,
        RobotParameters::k_driveMotorControllerKa,
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
        RobotParameters::k_driveMotorControllerKv,
        RobotParameters::k_driveMotorControllerKa,
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

    resetPose(Pose2D(Translation2D(0, 0), Rotation2D::fromDegrees(0)), Pose2D(Translation2D(0, 0), Rotation2D::fromDegrees(0)));

    double leftKsf;
    double rightKsf;

    SmartDashboard::PutNumber("leftKsf", leftKsf);
    SmartDashboard::PutNumber("rightKsf", rightKsf);
}

TankDrivetrain::~TankDrivetrain() {
    delete m_pLeftDriveMotor;
    m_pLeftDriveMotor = nullptr;

    delete m_pRightDriveMotor;
    m_pRightDriveMotor = nullptr;

    delete m_pLeftDriveMotorController;
    m_pLeftDriveMotorController = nullptr;

    delete m_pRightDriveMotorController;
    m_pRightDriveMotorController = nullptr;

    delete m_pLeftDriveEncoder;
    m_pLeftDriveEncoder = nullptr;

    delete m_pRightDriveEncoder;
    m_pRightDriveEncoder = nullptr;

    delete m_pLeftDriveMotorSlave;
    m_pLeftDriveMotorSlave = nullptr;

    delete m_pRightDriveMotorSlave;
    m_pRightDriveMotorSlave = nullptr;

    delete m_pShifter;
    m_pShifter = nullptr;

    delete m_pChassisIMU;
    m_pChassisIMU = nullptr;
}

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
    double robotAccel) {

    // get sign of vel
    int velSign = Sign::sign(robotVel);
    
    // limit robot vel
    if(robotVel > RobotParameters::k_maxSpeed) {
        robotVel = RobotParameters::k_maxSpeed;
    }
    else if(robotVel < -RobotParameters::k_maxSpeed) {
        robotVel = -RobotParameters::k_maxSpeed;
    }

    // limit robot accel
    if(robotAccel > RobotParameters::k_maxAccel) {
        robotAccel = RobotParameters::k_maxAccel;
    }
    else if(robotAccel < RobotParameters::k_maxDeccel) {
        robotAccel = RobotParameters::k_maxDeccel;
    }

//    // limit centrip accel
//    if(fabs(robotVel * robotYawRate * 180.0 / M_PI) > RobotParameters::k_maxCentripAccel) {
//        robotVel = velSign * RobotParameters::k_maxCentripAccel / fabs(robotYawRate * 180.0 / M_PI);
//    }

    // convert robot vel to wheel vel
    m_kinematics.inverseKinematics(robotVel, robotYawRate, m_leftWheelVelCmd, m_rightWheelVelCmd);

    // limit wheel vel
    if((fabs(m_leftWheelVelCmd) > RobotParameters::k_maxSpeed) || (fabs(m_rightWheelVelCmd) > RobotParameters::k_maxSpeed)) {
        if(robotVel >= 0) { // moving forward
            if(robotYawRate >= 0) { // turning left
                m_rightWheelVelCmd = RobotParameters::k_maxSpeed;
                m_leftWheelVelCmd = m_rightWheelVelCmd - RobotParameters::k_wheelTrack * robotYawRate * 180.0 / M_PI;
            }
            else { // turning right
                m_leftWheelVelCmd = RobotParameters::k_maxSpeed;
                m_rightWheelVelCmd = m_leftWheelVelCmd + RobotParameters::k_wheelTrack * robotYawRate * 180.0 / M_PI;
            }
        }
        else { // moving backward
            if(robotYawRate <= 0) { // turning left
                m_rightWheelVelCmd = -RobotParameters::k_maxSpeed;
                m_leftWheelVelCmd = m_rightWheelVelCmd - RobotParameters::k_wheelTrack * robotYawRate * 180.0 / M_PI;
            }
            else { // turning right
                m_leftWheelVelCmd = -RobotParameters::k_maxSpeed;
                m_rightWheelVelCmd = m_leftWheelVelCmd + RobotParameters::k_wheelTrack * robotYawRate * 180.0 / M_PI;
            }
        }
    }

    // convert wheel vel from translational to rotational
    double leftWheelAngVel = m_leftWheelVelCmd / RobotParameters::k_wheelRad * 180.0 / M_PI;
    double rightWheelAngVel = m_rightWheelVelCmd / RobotParameters::k_wheelRad * 180.0 / M_PI;
    double leftWheelAngAccel = robotAccel / RobotParameters::k_wheelRad * 180.0 / M_PI;
    double rightWheelAngAccel = robotAccel / RobotParameters::k_wheelRad * 180.0 / M_PI;

    // update motor vel controller
    m_pLeftDriveMotorController->updateClosedLoopControl(leftWheelAngVel, leftWheelAngAccel, .09);
    m_pRightDriveMotorController->updateClosedLoopControl(rightWheelAngVel, rightWheelAngAccel, .09);
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
		m_pLeftDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioHigh);
		m_pRightDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
	}

	return isHighGear;
}

Pose2D TankDrivetrain::getPose() {
    return m_tankDrivePose.getPose();
}

Pose2D TankDrivetrain::getPoseDot() {
    return m_tankDrivePose.getPoseDot();
}

void TankDrivetrain::updatePose() {
    // read left wheel encoder
    double oldLeftWheelDist = m_leftWheelDist;
    m_leftWheelDist = m_pLeftDriveEncoder->getWheelDistance(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
    SmartDashboard::PutNumber("leftWheelDist", m_leftWheelDist);
    double deltaDistLeftWheel = m_leftWheelDist - oldLeftWheelDist;
    double velLeftWheel = m_pLeftDriveEncoder->getWheelVelocity(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
    SmartDashboard::PutNumber("velLeftWheel", velLeftWheel);

//    // check for wheel slip
//    if(fabs(velLeftWheel) > fabs(m_leftWheelVelCmd * RobotParameters::k_wheelSlipNoiseRatio)) {
//            // account for sample time and measurement noise
//        deltaDistLeftWheel = m_leftWheelVelCmd * 1.0 / (double)RobotParameters::k_updateRate;
//        velLeftWheel = m_leftWheelVelCmd;
//    }

    // read right wheel encoder
    double oldRightWheelDist = m_rightWheelDist;
    m_rightWheelDist = m_pRightDriveEncoder->getWheelDistance(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
    SmartDashboard::PutNumber("rightWheelDist", m_rightWheelDist);
    double deltaDistRightWheel = m_rightWheelDist - oldRightWheelDist;
    double velRightWheel = m_pRightDriveEncoder->getWheelVelocity(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
    SmartDashboard::PutNumber("velRightWheel", velRightWheel);

    SmartDashboard::PutNumber("leftTalonVel", m_pLeftDriveEncoder->getTickVelocity() / 10);
    SmartDashboard::PutNumber("rightTalonVel", m_pRightDriveEncoder->getTickVelocity() / 10);

    SmartDashboard::PutNumber("talonVel", ((m_pLeftDriveEncoder->getTickVelocity() / 10) + (m_pRightDriveEncoder->getTickVelocity() / 10)) / 2);


//    // check for wheel slip
//    if(fabs(velRightWheel) > fabs(m_rightWheelVelCmd * RobotParameters::k_wheelSlipNoiseRatio)) {
//            // account for sample time and measurement noise
//        deltaDistRightWheel = m_rightWheelVelCmd * 1.0 / (double)RobotParameters::k_updateRate;
//        velRightWheel = m_rightWheelVelCmd;
//    }

    // read IMU
    double oldGyroYaw = m_gyroYaw;
    m_gyroYaw = -m_pChassisIMU->GetYaw();
    SmartDashboard::PutNumber("gyroYaw", m_gyroYaw);
    double deltaGyroYaw = m_gyroYaw - oldGyroYaw;
    double gyroYawRate = -m_pChassisIMU->GetRate();
    SmartDashboard::PutNumber("yawRateGyro", gyroYawRate);

    // update pose
    m_tankDrivePose.update(deltaDistLeftWheel, deltaDistRightWheel, deltaGyroYaw, velLeftWheel, velRightWheel, gyroYawRate);

    SmartDashboard::PutNumber("x", m_tankDrivePose.getPose().getTranslation().getX());
    SmartDashboard::PutNumber("y", m_tankDrivePose.getPose().getTranslation().getY());
    SmartDashboard::PutNumber("yaw", m_tankDrivePose.getPose().getRotation().getDegrees());

    SmartDashboard::PutNumber("xVel", m_tankDrivePose.getPoseDot().getTranslation().getX());
    SmartDashboard::PutNumber("yVel", m_tankDrivePose.getPoseDot().getTranslation().getY());
	SmartDashboard::PutNumber("yawRate", m_tankDrivePose.getPoseDot().getRotation().getDegrees());
}

void TankDrivetrain::resetPose(const Pose2D &pose, const Pose2D &poseDot) {
    m_tankDrivePose.reset(pose, poseDot);
}

void TankDrivetrain::zeroDriveEncoders() {
	m_pLeftDriveEncoder->zero();
	m_pRightDriveEncoder->zero();
}

void TankDrivetrain::zeroGyroYaw() {
	m_pChassisIMU->ZeroYaw();
}
