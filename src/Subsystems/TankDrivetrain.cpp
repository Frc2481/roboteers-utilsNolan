#include "TankDrivetrain.h"
#include "RobotMap.h"
#include "RobotParameters.h"
#include "Commands/TankDrivetrainJoystickDrive.h"

TankDrivetrain::TankDrivetrain()
    : Subsystem("TankDrivetrain"),
    m_wheelRad(RobotParameters::k_wheelRad),
    m_wheelTrack(RobotParameters::k_wheelTrack),
    m_kinematics(RobotParameters::k_wheelTrack),
    m_maxSpeed(RobotParameters::k_maxSpeed),
    m_maxAccel(RobotParameters::k_maxAccel),
    m_maxDeccel(RobotParameters::k_maxDeccel),
    m_maxCentripAccel(RobotParameters::k_maxCentripAccel),
    m_updateRate(RobotParameters::k_updateRate),
    m_tankDrivePose(Pose2D(0, 0), RobotParameters::k_wheelTrack, RobotParameters::k_cornerStiffCoeff),
    m_driveGearRatio(RobotParameters::k_driveGearRatio),
    m_leftWheelDist(0),
    m_rightWheelDist(0),
    m_leftWheelVelCmd(0),
    m_rightWheelVelCmd(0),
    m_gyroYaw(0) {
    
    m_pLeftDriveMotor = new TalonSRX(LEFT_DRIVE_MOTOR_ID);
    m_pLeftDriveMotorController = new MotorVelocityController(
        m_pLeftDriveMotor,
        false,
        RobotParameters::k_driveMotorControllerKp,
        RobotParameters::k_driveMotorControllerKi,
        RobotParameters::k_driveMotorControllerKd,
        RobotParameters::k_driveMotorControllerKv,
        RobotParameters::k_driveMotorControllerKa,
        0,
        0,
        RobotParameters::k_grayhillEncoderTicksPerRev);
    m_pLeftDriveEncoder = new GrayhillEncoder(m_pLeftDriveMotorController, "LEFT_DRIVE_MOTOR_ENCODER");
    m_pLeftDriveMotorSlave = new TalonSRX(LEFT_DRIVE_MOTOR_SLAVE_ID);
    m_pLeftDriveMotorSlave->Set(ControlMode::Follower, LEFT_DRIVE_MOTOR_ID);
    
    m_pRightDriveMotor = new TalonSRX(RIGHT_DRIVE_MOTOR_ID);
    m_pRightDriveMotorController = new MotorVelocityController(
        m_pRightDriveMotor,
        true,
        RobotParameters::k_driveMotorControllerKp,
        RobotParameters::k_driveMotorControllerKi,
        RobotParameters::k_driveMotorControllerKd,
        RobotParameters::k_driveMotorControllerKv,
        RobotParameters::k_driveMotorControllerKa,
        0,
        0,
        RobotParameters::k_grayhillEncoderTicksPerRev);
    m_pRightDriveEncoder = new GrayhillEncoder(m_pRightDriveMotor, "RIGHT_DRIVE_MOTOR_ENCODER");
    m_pRightDriveMotorSlave = new TalonSRX(RIGHT_DRIVE_MOTOR_SLAVE_ID);
    m_pRightDriveMotorSlave->Set(ControlMode::Follower, RIGHT_DRIVE_MOTOR_ID);

    m_pChassisIMU = new AHRS(SPI::kMXP);

    resetPose(Pose2D(0, 0));
}

void TankDrivetrain::~TankDrivetrain() {
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
    
    delete m_pChassisIMU;
    m_pChassisIMU = nullptr;
    
}

void TankDrivetrain::InitDefaultCommand() {
	SetDefaultCommand(new TankDrivetrainJoystickDrive);
}

void TankDrivetrain::Periodic() {
    // update pose
    updatePose();
}

void TankDrivetrain::driveOpenLoopControl(const double &percentLeftDrive, const double &percentRightDrive) {
    m_pLeftDriveMotorController->updateOpenLoopControl(percentLeftDrive);
    m_pRightDriveMotorController->updateOpenLoopControl(percentRightDrive);
}

void TankDrivetrain::driveClosedLoopControl(
    const double &robotVel,
    const double &robotYawRate,
    const double &robotAccel) {

    // get sign of vel
    int velSign = sign(robotVel);
    
    // limit robot vel
    if(robotVel > m_maxSpeed) {
        robotVel = m_maxSpeed;
    }
    else if(robotVel < -m_maxSpeed) {
        robotVel = -m_maxSpeed;
    }

    // limit robot accel
    if(robotAccel > m_maxAccel) {
        robotAccel = m_maxAccel;
    }
    else if(robotAccel < m_maxDeccel) {
        robotAccel = m_maxDeccel;
    }

    // limit centrip accel
    if(fabs(robotVel * robotYawRate * 180.0 / M_PI) > m_maxCentripAccel) {
        robotVel = velSign * m_maxCentripAccel / fabs(robotYawRate * 180.0 / M_PI);
    }

    // convert robot vel to wheel vel
    m_kinematics.inverseKinematics(robotVel, robotYawRate, m_leftWheelVelCmd, m_rightWheelVelCmd);

    // limit wheel vel
    if((fabs(m_leftWheelVelCmd) > m_maxSpeed) || (fabs(m_rightWheelVelCmd) > m_maxSpeed)) {
        if(robotVel >= 0) { // moving forward
            if(robotYawRate >= 0) { // turning left
                m_rightWheelVelCmd = m_maxSpeed;
                m_leftWheelVelCmd = m_rightWheelVelCmd - m_wheelTrack * robotYawRate * 180.0 / M_PI;
            }
            else { // turning right
                m_leftWheelVelCmd = m_maxSpeed;
                m_rightWheelVelCmd = m_leftWheelVelCmd + m_wheelTrack * robotYawRate * 180.0 / M_PI;
            }
        }
        else { // moving backward
            if(robotYawRate <= 0) { // turning left
                m_rightWheelVelCmd = -m_maxSpeed;
                m_leftWheelVelCmd = m_rightWheelVelCmd - m_wheelTrack * robotYawRate * 180.0 / M_PI;
            }
            else { // turning right
                m_leftWheelVelCmd = -m_maxSpeed;
                m_rightWheelVelCmd = m_leftWheelVelCmd + m_wheelTrack * robotYawRate * 180.0 / M_PI;
            }
        }
    }

    // convert wheel vel from translational to rotational
    double leftWheelAngVel = m_leftWheelVelCmd / m_wheelRad * 180.0 / M_PI;
    double righttWheelAngVel = m_rightWheelVelCmd / m_wheelRad * 180.0 / M_PI;
    double leftWheelAngAccel = robotAccel / m_wheelRad * 180.0 / M_PI;
    double rightWheelAngAccel = robotAccel / m_wheelRad * 180.0 / M_PI;
    
    // update motor vel controller
    m_pLeftDriveMotorController->updateClosedLoopControl(leftWheelAngVel, leftWheelAngAccel);
    m_pRightDriveMotorController->updateClosedLoopControl(righttWheelAngVel, rightWheelAngAccel);
}

void TankDrivetrain::stop() {
    driveOpenLoopControl(0, 0);
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
    m_leftWheelDist = m_pLeftDriveEncoder->getWheelDistance(m_wheelRad, m_driveGearRatio);
    double deltaDistLeftWheel = m_leftWheelDist - oldLeftWheelDist;
    double velLeftWheel = m_pLeftDriveEncoder->getWheelVelocity(m_wheelRad, m_driveGearRatio);

    // check for wheel slip
    if(fabs(velLeftWheel) > fabs(m_leftWheelVelCmd * RobotParameters::k_wheelSlipNoiseRatio)) {
            // account for sample time and measurement noise
        deltaDistLeftWheel = m_leftWheelVelCmd * 1.0 / (double)m_updateRate;
        velLeftWheel = m_leftWheelVelCmd;
    }

    // read right wheel encoder
    double oldRightWheelDist = m_rightWheelDist;
    m_rightWheelDist = m_pRightDriveEncoder->getWheelDistance(m_wheelRad, m_driveGearRatio);
    double deltaDistRightWheel = m_rightWheelDist - oldRightWheelDist;
    double velRightWheel = m_pRightDriveEncoder->getWheelVelocity(m_wheelRad, m_driveGearRatio);

    // check for wheel slip
    if(fabs(velRightWheel) > fabs(m_rightWheelVelCmd * RobotParameters::k_wheelSlipNoiseRatio)) {
            // account for sample time and measurement noise
        deltaDistRightWheel = m_rightWheelVelCmd * 1.0 / (double)m_updateRate;
        velRightWheel = m_rightWheelVelCmd;
    }

    // read IMU
    double oldGyroYaw = m_gyroYaw;
    m_gyroYaw = -m_pChassisIMU->GetYaw();
    double deltaYawGyro = m_gyroYaw - oldGyroYaw;
    double yawRateGyro = -m_pChassisIMU->GetRate();

    // update pose
    m_tankDrivePose.update(deltaDistLeftWheel, deltaDistRightWheel, deltaYawGyro, velLeftWheel, velRightWheel, yawRateGyro);
}

void TankDrivetrain::resetPose(const Pose2D &pose, const Pose2D &poseDot) {
    m_tankDrivePose.reset(pose, poseDot);
    m_pLeftDriveEncoder->zero();
    m_leftWheelDist = 0;
    m_pRightDriveEncoder->zero();
    m_rightWheelDist = 0;
    m_pChassisIMU->ZeroYaw();
    m_gyroYaw = 0;
}
