#include "TankDrivetrain.h"

TankDrivetrain::TankDrivetrain()
    : Subsystem("TankDrivetrain"),
    m_wheelRad(wheelRad),
    m_wheelTrack(wheelTrack),
    m_kinematics(wheelTrack),
    m_maxSpeed(maxSpeed),
    m_maxAccel(maxAccel),
    m_maxDeccel(maxDeccel),
    m_maxCentripAccel(maxCentripAccel),
    m_updateRate(updateRate),
    m_tankDrivePose(Pose2D(0, 0), wheelTrack, cornerStiffCoeff),
    m_driveGearRatio(driveGearRatio),
    m_leftWheelDist(0),
    m_rightWheelDist(0),
    m_leftWheelVelCmd(0),
    m_rightWheelVelCmd(0),
    m_gyroYaw(0) {
    
    m_pLeftDriveMotor = new TalonSRX(leftDriveMotorID);
    m_pLeftDriveMotorController = new MotorVelocityController(
        m_pLeftDriveMotor,
        kp,
        ki,
        kd,
        kv,
        ka,
        iZone,
        iErrorLim,
        ticksPerRev);
    m_pLeftDriveEncoder = new GrayhillEncoder(m_pLeftDriveMotorController, leftDriveEncoderName);
    
    m_pRightDriveMotor = new TalonSRX(rightDriveMotorID);
    m_pRightDriveMotorController = new MotorVelocityController(
        m_pRightDriveMotor,
        kp,
        ki,
        kd,
        kv,
        ka,
        iZone,
        iErrorLim,
        ticksPerRev);
    m_pRightDriveEncoder = new GrayhillEncoder(m_pRightDriveMotor, rightDriveEncoderName);

    m_pChassisIMU = new AHRS(SPI::kMXP);

    resetPose(Pose2D(0, 0));
}

TankDrivetrain::~TankDrivetrain() {
}

void TankDrivetrain::InitDefaultCommand() {
	SetDefaultCommand(new defaultCommand);
}

void TankDrivetrain::Periodic() {
    // update pose
    updatePose();
}

void TankDrivetrain::drive(
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

    double deltaTime = time - m_oldTime;
    double limitVelHigh;
    double limitVelLow;

    double avgAccel = (m_maxAccel - m_maxDeccel) * updateRate / 2.0;
    if(fabs(robotVel) < avgAccel) { // account for behavior around zero
        limitVelHigh = avgAccel;
        limitVelLow = -avgAccel;
    }
    if(velSign > 0) {
        limitVelHigh = m_oldVel + m_maxAccel / (double)updateRate;
        limitVelLow = m_oldVel + m_maxDeccel / (double)updateRate;
    }
    else if(velSign < 0) {
        limitVelHigh = m_oldVel - m_maxAccel / (double)updateRate;
        limitVelLow = m_oldVel - m_maxDeccel / (double)updateRate;
    }

    if(robotVel > limitVelHigh) {
        robotVel = limitVelHigh;
    }
    else if(robotVel < limitVelLow) {
        robotVel = limitVelLow;
    }

    // limit centrip accel
    if(fabs(robotVel * robotYawRate * 180.0 / M_PI) > m_maxCentripAccel) {
        robotVel = velSign * m_maxCentripAccel / fabs(robotYawRate * 180.0 / M_PI);
    }

    // convert robot vel to wheel vel
    m_kinematics.inverseKinematics(robotVel, robotYawRate, m_leftWheelVelCmd, m_rightWheelVelCmd);

    // limit wheel vel
    if(m_leftWheelVelCmd > m_maxSpeed) {
        m_leftWheelVelCmd = m_maxSpeed;
        m_rightWheelVelCmd = m_leftWheelVelCmd - m_wheelTrack * robotYawRate * 180.0 / M_PI;
    }
    else if (m_leftWheelVelCmd < - m_maxSpeed) {
        m_maxSpeed = -m_maxSpeed;
        m_rightWheelVelCmd = m_leftWheelVelCmd - m_wheelTrack * robotYawRate * 180.0 / M_PI;
    }
    if(m_rightWheelVelCmd > m_maxSpeed) {
        m_rightWheelVelCmd = m_maxSpeed;
        m_leftWheelVelCmd = m_rightWheelVelCmd + m_wheelTrack * robotYawRate * 180.0 / M_PI;
    }
    else if (m_rightWheelVelCmd < - m_maxSpeed) {
        m_rightWheelVelCmd = -m_maxSpeed;
        m_rightWheelVelCmd = m_rightWheelVelCmd + m_wheelTrack * robotYawRate * 180.0 / M_PI;
    }

    // convert wheel vel from translational to rotational
    double leftWheelAngVel = m_leftWheelVelCmd / m_wheelRad * 180.0 / M_PI;
    double righttWheelAngVel = m_rightWheelVelCmd / m_wheelRad * 180.0 / M_PI;
    double leftWheelAngAccel = robotAccel / m_wheelRad * 180.0 / M_PI;
    double rightWheelAngAccel = robotAccel / m_wheelRad * 180.0 / M_PI;
    
    // update motor vel controller
    m_pLeftDriveMotorController->update(leftWheelAngVel, leftWheelAngAccel);
    m_pRightDriveMotorController->update(righttWheelAngVel, rightWheelAngAccel);
}

void TankDrivetrain::stop() {
    drive(0, 0, m_maxDeccel);
}

Pose2D TankDriveTrain::getPose() {
    m_tankDrivePose.get();
}

void TankDriveTrain::updatePose() {
    // read left wheel encoder
    double oldLeftWheelDist = m_leftWheelDist;
    m_leftWheelDist = m_pLeftDriveEncoder->getWheelDistance(m_wheelRad, m_driveGearRatio);
    double deltaDistLeftWheel = m_leftWheelDist - oldLeftWheelDist;

    // check for wheel slip
    if(fabs(deltaDistLeftWheel) > fabs(m_leftWheelVelCmd * ROBOT_SCHEDULER_PERIOD * WHEEL_SLIP_NOISE_RATIO)) {
            // account for sample time and measurement noise
        deltaDistLeftWheel = m_leftWheelVelCmd * ROBOT_SCHEDULER_PERIOD;
    }

    // read right wheel encoder
    double oldRightWheelDist = m_rightWheelDist;
    m_rightWheelDist = m_pRightDriveEncoder->getWheelDistance(m_wheelRad, m_driveGearRatio);
    double deltaDistRightWheel = m_rightWheelDist - oldRightWheelDist;

    // check for wheel slip
    if(fabs(deltaDistRightWheel) > fabs(m_rightWheelVelCmd * ROBOT_SCHEDULER_PERIOD * WHEEL_SLIP_NOISE_RATIO)) {
            // account for sample time and measurement noise
        deltaDistRightWheel = m_rightWheelVelCmd * ROBOT_SCHEDULER_PERIOD;
    }

    // read IMU
    double oldGyroYaw = m_gyroYaw;
    m_gyroYaw = -m_pChassisIMU->GetYaw();
    double deltaYawGyro = m_gyroYaw - oldGyroYaw;

    // update pose
    m_tankDrivePose.update(deltaDistLeftWheel, deltaDistRightWheel, deltaYawGyro);
}

void TankDriveTrain::resetPose(const Pose2D &pose) {
    m_tankDrivePose.reset(pose);
    m_pLeftDriveEncoder->zero();
    m_leftWheelDist = 0;
    m_pRightDriveEncoder->zero();
    m_rightWheelDist = 0;
    m_pChassisIMU->ZeroYaw();
    m_gyroYaw = 0;
}
