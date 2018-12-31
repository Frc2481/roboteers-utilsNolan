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
    double robotAccel,
	double robotYawAccel) {

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

    // limit robot yaw rate
    if(robotYawRate > (RobotParameters::k_maxSpeed / (RobotParameters::k_wheelTrack / 2.0) * 180.0 / M_PI)) {
    	robotYawRate = (RobotParameters::k_maxSpeed / (RobotParameters::k_wheelTrack / 2.0) * 180.0 / M_PI);
	}
	else if(robotYawRate < -(RobotParameters::k_maxSpeed / (RobotParameters::k_wheelTrack / 2.0) * 180.0 / M_PI)) {
		robotYawRate = -(RobotParameters::k_maxSpeed / (RobotParameters::k_wheelTrack / 2.0) * 180.0 / M_PI);
	}

    // limit robot yaw accel
    if(robotYawAccel > (RobotParameters::k_maxAccel / (RobotParameters::k_wheelTrack / 2.0) * 180.0 / M_PI)) {
		robotYawAccel = (RobotParameters::k_maxAccel / (RobotParameters::k_wheelTrack / 2.0) * 180.0 / M_PI);
	}
	else if(robotYawAccel < -(RobotParameters::k_maxDeccel / (RobotParameters::k_wheelTrack / 2.0) * 180.0 / M_PI)) {
		robotYawAccel = -(RobotParameters::k_maxDeccel / (RobotParameters::k_wheelTrack / 2.0) * 180.0 / M_PI);
	}

//    // limit centrip accel
//    if(fabs(robotVel * robotYawRate * 180.0 / M_PI) > RobotParameters::k_maxCentripAccel) {
//        robotVel = Sign::sign(robotVel) * RobotParameters::k_maxCentripAccel / fabs(robotYawRate * 180.0 / M_PI);
//    }

    // convert robot vel to wheel vel
    double leftWheelVelCmd;
	double rightWheelVelCmd;
    m_kinematics.inverseKinematics(robotVel, robotYawRate, leftWheelVelCmd, rightWheelVelCmd);

    // limit wheel vel
    if((fabs(leftWheelVelCmd) > RobotParameters::k_maxSpeed) || (fabs(rightWheelVelCmd) > RobotParameters::k_maxSpeed)) {
        if(robotVel >= 0) { // moving forward
            if(robotYawRate >= 0) { // turning left
                rightWheelVelCmd = RobotParameters::k_maxSpeed;
                leftWheelVelCmd = rightWheelVelCmd - RobotParameters::k_wheelTrack * robotYawRate * M_PI / 180.0;
            }
            else { // turning right
                leftWheelVelCmd = RobotParameters::k_maxSpeed;
                rightWheelVelCmd = leftWheelVelCmd + RobotParameters::k_wheelTrack * robotYawRate * M_PI / 180.0;
            }
        }
        else { // moving backward
            if(robotYawRate <= 0) { // turning left
                rightWheelVelCmd = -RobotParameters::k_maxSpeed;
                leftWheelVelCmd = rightWheelVelCmd - RobotParameters::k_wheelTrack * robotYawRate * M_PI / 180.0;
            }
            else { // turning right
                leftWheelVelCmd = -RobotParameters::k_maxSpeed;
                rightWheelVelCmd = leftWheelVelCmd + RobotParameters::k_wheelTrack * robotYawRate * M_PI / 180.0;
            }
        }
    }

    // convert robot accel to wheel accel
	double leftWheelAccelCmd;
	double rightWheelAccelCmd;
	m_kinematics.inverseKinematics(robotAccel, robotYawAccel, leftWheelAccelCmd, rightWheelAccelCmd);

	// limit wheel accel
	if((fabs(leftWheelAccelCmd) > RobotParameters::k_maxAccel) || (fabs(rightWheelAccelCmd) > RobotParameters::k_maxAccel)) {
		if(robotAccel >= 0) { // accel forward
			if(robotYawAccel >= 0) { // accel left
				rightWheelAccelCmd = RobotParameters::k_maxAccel;
				leftWheelAccelCmd = rightWheelAccelCmd - RobotParameters::k_wheelTrack * robotYawAccel * M_PI / 180.0;
			}
			else { // accel right
				leftWheelAccelCmd = RobotParameters::k_maxAccel;
				rightWheelAccelCmd = leftWheelAccelCmd + RobotParameters::k_wheelTrack * robotYawAccel * M_PI / 180.0;
			}
		}
		else { // accel backward
			if(robotYawAccel <= 0) { // accel left
				rightWheelAccelCmd = -RobotParameters::k_maxAccel;
				leftWheelAccelCmd = rightWheelAccelCmd - RobotParameters::k_wheelTrack * robotYawAccel * M_PI / 180.0;
			}
			else { // accel right
				leftWheelAccelCmd = -RobotParameters::k_maxAccel;
				rightWheelAccelCmd = leftWheelAccelCmd + RobotParameters::k_wheelTrack * robotYawAccel * M_PI / 180.0;
			}
		}
	}

    // convert wheel vel from translational to rotational
    double leftWheelAngVel = leftWheelVelCmd / RobotParameters::k_wheelRad * 180.0 / M_PI;
    double rightWheelAngVel = rightWheelVelCmd / RobotParameters::k_wheelRad * 180.0 / M_PI;
    double leftWheelAngAccel = robotAccel / RobotParameters::k_wheelRad * 180.0 / M_PI;
    double rightWheelAngAccel = robotAccel / RobotParameters::k_wheelRad * 180.0 / M_PI;

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
    double rightWheelVel = m_pRightDriveEncoder->getWheelVelocity(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
    SmartDashboard::PutNumber("rightWheelVel", rightWheelVel);

//    // check for wheel slip
//    if(fabs(velRightWheel) > fabs(m_rightWheelVelCmd * RobotParameters::k_wheelSlipNoiseRatio)) {
//            // account for sample time and measurement noise
//        deltaDistRightWheel = m_rightWheelVelCmd * 1.0 / (double)RobotParameters::k_updateRate;
//        velRightWheel = m_rightWheelVelCmd;
//    }

    // read IMU
    double oldGyroYaw = m_gyroYaw;
    m_gyroYaw = -m_pChassisIMU->GetYaw();
    double deltaGyroYaw = m_gyroYaw - oldGyroYaw;
    double gyroYawRate = -m_pChassisIMU->GetRate() * 180.0 / M_PI;
    SmartDashboard::PutNumber("yawRateGyro", gyroYawRate);

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
