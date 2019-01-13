// #include "SwerveDrivetrain.h"
// #include "../RobotMap.h"
// #include "../RobotParameters.h"
// #include "../Commands/SwerveDrivetrainJoystickDrive.h"
// #include "../Utils/Sign.h"
// #include "../Utils/MathConstants.h"

// SwerveDrivetrain::SwerveDrivetrain()
//     : Subsystem("SwerveDrivetrain"),
// 	m_kinematics(
// 		Translation2D(RobotParameters::k_wheelTrack / 2.0, RobotParameters::k_wheelBase / 2.0),
// 		Translation2D(RobotParameters::k_wheelTrack / 2.0, -RobotParameters::k_wheelBase / 2.0),
// 		Translation2D(-RobotParameters::k_wheelTrack / 2.0, -RobotParameters::k_wheelBase / 2.0),
// 		Translation2D(-RobotParameters::k_wheelTrack / 2.0, RobotParameters::k_wheelBase / 2.0)),
//     m_swerveDrivePose(
//     		Pose2D(Translation2D(0, 0), Rotation2D::fromDegrees(0)),
// 			RobotParameters::k_wheelTrack,
// 			RobotParameters::k_wheelBase,
// 			RobotParameters::k_cornerStiffCoeff),
// 	m_frWheelDist(0),
//     m_brWheelDist(0),
//     m_blWheelDist(0),
//     m_flWheelDist(0),
//     m_gyroYaw(0) {
    
// 	m_pFRDriveMotor = new TalonSRX(FR_DRIVE_MOTOR_ID);
// 	m_pFRDriveMotorController = new MotorVelocityController(
// 		m_pFRDriveMotor,
// 		false,
//         RobotParameters::k_driveMotorControllerKp,
//         RobotParameters::k_driveMotorControllerKi,
//         RobotParameters::k_driveMotorControllerKd,
//         RobotParameters::k_driveMotorControllerKv,
//         RobotParameters::k_driveMotorControllerKap,
// 		RobotParameters::k_driveMotorControllerKan,
// 		RobotParameters::k_driveMotorControllerKsf,
//         0,
// 		0,
//         RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
// 	m_pFRDriveEncoder = new GrayhillEncoder(m_pFRDriveMotor, "FR_DRIVE_MOTOR_ENCODER");

// 	m_pBRDriveMotor = new TalonSRX(BR_DRIVE_MOTOR_ID);
// 	m_pBRDriveMotorController = new MotorVelocityController(
// 		m_pBRDriveMotor,
// 		false,
// 		RobotParameters::k_driveMotorControllerKp,
// 		RobotParameters::k_driveMotorControllerKi,
// 		RobotParameters::k_driveMotorControllerKd,
// 		RobotParameters::k_driveMotorControllerKv,
// 		RobotParameters::k_driveMotorControllerKap,
// 		RobotParameters::k_driveMotorControllerKan,
// 		RobotParameters::k_driveMotorControllerKsf,
// 		0,
// 		0,
// 		RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
// 	m_pBRDriveEncoder = new GrayhillEncoder(m_pBRDriveMotor, "BR_DRIVE_MOTOR_ENCODER");

// 	m_pBLDriveMotor = new TalonSRX(BL_DRIVE_MOTOR_ID);
// 	m_pBLDriveMotorController = new MotorVelocityController(
// 		m_pBLDriveMotor,
// 		false,
// 		RobotParameters::k_driveMotorControllerKp,
// 		RobotParameters::k_driveMotorControllerKi,
// 		RobotParameters::k_driveMotorControllerKd,
// 		RobotParameters::k_driveMotorControllerKv,
// 		RobotParameters::k_driveMotorControllerKap,
// 		RobotParameters::k_driveMotorControllerKan,
// 		RobotParameters::k_driveMotorControllerKsf,
// 		0,
// 		0,
// 		RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
// 	m_pBLDriveEncoder = new GrayhillEncoder(m_pBLDriveMotor, "BL_DRIVE_MOTOR_ENCODER");

// 	m_pFLDriveMotor = new TalonSRX(FL_DRIVE_MOTOR_ID);
// 	m_pFLDriveMotorController = new MotorVelocityController(
// 		m_pFLDriveMotor,
// 		false,
// 		RobotParameters::k_driveMotorControllerKp,
// 		RobotParameters::k_driveMotorControllerKi,
// 		RobotParameters::k_driveMotorControllerKd,
// 		RobotParameters::k_driveMotorControllerKv,
// 		RobotParameters::k_driveMotorControllerKap,
// 		RobotParameters::k_driveMotorControllerKan,
// 		RobotParameters::k_driveMotorControllerKsf,
// 		0,
// 		0,
// 		RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
// 	m_pFLDriveEncoder = new GrayhillEncoder(m_pFLDriveMotor, "FL_DRIVE_MOTOR_ENCODER");

// 	m_pFRSteerMotor = new TalonSRX(FR_STEER_MOTOR_ID);
// 	m_pFRSteerMotorController = new MotorPositionController(
// 		m_pFRSteerMotor,
// 		false,
// 		RobotParameters::k_steerMotorControllerKp,
// 		RobotParameters::k_steerMotorControllerKi,
// 		RobotParameters::k_steerMotorControllerKd,
// 		RobotParameters::k_steerMotorControllerKv,
// 		RobotParameters::k_steerMotorControllerKap,
// 		RobotParameters::k_steerMotorControllerKan,
// 		RobotParameters::k_steerMotorControllerKsf,
// 		0,
// 		0,
// 		RobotParameters::k_ctreMagEncoderTicksPerRev);
// 	m_pFRSteerEncoder = new CTREMagEncoder(m_pFRSteerMotor, "FR_STEER_MOTOR_ENCODER");

// 	m_pBRSteerMotor = new TalonSRX(BR_STEER_MOTOR_ID);
// 	m_pBRSteerMotorController = new MotorPositionController(
// 		m_pBRSteerMotor,
// 		false,
// 		RobotParameters::k_steerMotorControllerKp,
// 		RobotParameters::k_steerMotorControllerKi,
// 		RobotParameters::k_steerMotorControllerKd,
// 		RobotParameters::k_steerMotorControllerKv,
// 		RobotParameters::k_steerMotorControllerKap,
// 		RobotParameters::k_steerMotorControllerKan,
// 		RobotParameters::k_steerMotorControllerKsf,
// 		0,
// 		0,
// 		RobotParameters::k_ctreMagEncoderTicksPerRev);
// 	m_pBRSteerEncoder = new CTREMagEncoder(m_pBRSteerMotor, "BR_STEER_MOTOR_ENCODER");

// 	m_pBLSteerMotor = new TalonSRX(BL_STEER_MOTOR_ID);
// 	m_pBLSteerMotorController = new MotorPositionController(
// 		m_pBLSteerMotor,
// 		false,
// 		RobotParameters::k_steerMotorControllerKp,
// 		RobotParameters::k_steerMotorControllerKi,
// 		RobotParameters::k_steerMotorControllerKd,
// 		RobotParameters::k_steerMotorControllerKv,
// 		RobotParameters::k_steerMotorControllerKap,
// 		RobotParameters::k_steerMotorControllerKan,
// 		RobotParameters::k_steerMotorControllerKsf,
// 		0,
// 		0,
// 		RobotParameters::k_ctreMagEncoderTicksPerRev);
// 	m_pBLSteerEncoder = new CTREMagEncoder(m_pBLSteerMotor, "BL_STEER_MOTOR_ENCODER");

// 	m_pFLSteerMotor = new TalonSRX(FL_STEER_MOTOR_ID);
// 	m_pFLSteerMotorController = new MotorPositionController(
// 		m_pFLSteerMotor,
// 		false,
// 		RobotParameters::k_steerMotorControllerKp,
// 		RobotParameters::k_steerMotorControllerKi,
// 		RobotParameters::k_steerMotorControllerKd,
// 		RobotParameters::k_steerMotorControllerKv,
// 		RobotParameters::k_steerMotorControllerKap,
// 		RobotParameters::k_steerMotorControllerKan,
// 		RobotParameters::k_steerMotorControllerKsf,
// 		0,
// 		0,
// 		RobotParameters::k_ctreMagEncoderTicksPerRev);
// 	m_pFLSteerEncoder = new CTREMagEncoder(m_pFLSteerMotor, "FL_STEER_MOTOR_ENCODER");

// //    m_pShifter = new Solenoid(DRIVE_XMSN_SHIFTER_ID);
// //    setShiftState(false);

//     m_pChassisIMU = new AHRS(SPI::kMXP);

//     resetPose(Pose2D(Translation2D(0, 0), Rotation2D::fromDegrees(0)), PoseDot2D(0, 0, 0));
// }

// // SwerveDrivetrain::~SwerveDrivetrain() {
// // 	delete m_pFRDriveMotor;
// // 	m_pFRDriveMotor = nullptr;

// // 	delete m_pBRDriveMotor;
// // 	m_pBRDriveMotor = nullptr;

// // 	delete m_pBLDriveMotor;
// // 	m_pBLDriveMotor = nullptr;

// // 	delete m_pFLDriveMotor;
// // 	m_pFLDriveMotor = nullptr;

// // 	delete m_pFRDriveMotorController;
// // 	m_pFRDriveMotorController = nullptr;

// // 	delete m_pBRDriveMotorController;
// // 	m_pBRDriveMotorController = nullptr;

// // 	delete m_pBLDriveMotorController;
// // 	m_pBLDriveMotorController = nullptr;

// // 	delete m_pFLDriveMotorController;
// // 	m_pFLDriveMotorController = nullptr;

// // 	delete m_pFRDriveEncoder;
// // 	m_pFRDriveEncoder = nullptr;

// // 	delete m_pBRDriveEncoder;
// // 	m_pBRDriveEncoder = nullptr;

// // 	delete m_pBLDriveEncoder;
// // 	m_pBLDriveEncoder = nullptr;

// // 	delete m_pFLDriveEncoder;
// // 	m_pFLDriveEncoder = nullptr;

// // 	delete m_pFRSteerMotor;
// // 	m_pFRSteerMotor = nullptr;

// // 	delete m_pBRSteerMotor;
// // 	m_pBRSteerMotor = nullptr;

// // 	delete m_pBLSteerMotor;
// // 	m_pBLSteerMotor = nullptr;

// // 	delete m_pFLSteerMotor;
// // 	m_pFLSteerMotor = nullptr;

// // 	delete m_pFRSteerMotorController;
// // 	m_pFRSteerMotorController = nullptr;

// // 	delete m_pBRSteerMotorController;
// // 	m_pBRSteerMotorController = nullptr;

// // 	delete m_pBLSteerMotorController;
// // 	m_pBLSteerMotorController = nullptr;

// // 	delete m_pFLSteerMotorController;
// // 	m_pFLSteerMotorController = nullptr;

// // 	delete m_pFRSteerEncoder;
// // 	m_pFRSteerEncoder = nullptr;

// // 	delete m_pBRSteerEncoder;
// // 	m_pBRSteerEncoder = nullptr;

// // 	delete m_pBLSteerEncoder;
// // 	m_pBLSteerEncoder = nullptr;

// // 	delete m_pFLSteerEncoder;
// // 	m_pFLSteerEncoder = nullptr;

// //     delete m_pShifter;
// //     m_pShifter = nullptr;

// //     delete m_pChassisIMU;
// //     m_pChassisIMU = nullptr;
// // }

// void SwerveDrivetrain::InitDefaultCommand() {
// 	SetDefaultCommand(new SwerveDrivetrainJoystickDrive());
// }

// void SwerveDrivetrain::Periodic() {
// 	// update encoders
// 	m_pFRDriveEncoder->update();
// 	m_pBRDriveEncoder->update();
// 	m_pBLDriveEncoder->update();
// 	m_pFLDriveEncoder->update();
// 	m_pFRSteerEncoder->update();
// 	m_pBRSteerEncoder->update();
// 	m_pBLSteerEncoder->update();
// 	m_pFLSteerEncoder->update();

//     // update shift state
// 	getShiftState();

// 	// update pose
//     updatePose();
// }

// void SwerveDrivetrain::driveOpenLoopControl(double percentVelX, double percentVelY, double percentYawRate) {
// 	// convert percent to physical units
// 	double robotVelX = RobotParameters::k_maxSpeed * percentVelX;
// 	double robotVelY = RobotParameters::k_maxSpeed * percentVelY;
// 	double robotYawRate = (RobotParameters::k_maxSpeed / pow(pow(RobotParameters::k_wheelTrack / 2.0, 2) + pow(RobotParameters::k_wheelBase / 2.0, 2), 0.5)) * 180.0 / MATH_CONSTANTS_PI * percentYawRate;

// 	Translation2D frWheelVel;
// 	Translation2D brWheelVel;
// 	Translation2D blWheelVel;
// 	Translation2D flWheelVel;
// 	m_kinematics.inverseKinematics(
// 		Translation2D(robotVelX, robotVelY),
// 		robotYawRate,
// 		frWheelVel,
// 		brWheelVel,
// 		blWheelVel,
// 		flWheelVel);

// 	// convert physical units to percent
// 	frWheelVel.scaleBy(1 / RobotParameters::k_maxSpeed);
// 	brWheelVel.scaleBy(1 / RobotParameters::k_maxSpeed);
// 	blWheelVel.scaleBy(1 / RobotParameters::k_maxSpeed);
// 	flWheelVel.scaleBy(1 / RobotParameters::k_maxSpeed);

// 	// limit wheel vel
// 	double maxWheelSpeed = std::max(frWheelVel.norm(), std::max(brWheelVel.norm(), std::max(blWheelVel.norm(), flWheelVel.norm())));
// 	if(maxWheelSpeed > 1) {
// 		frWheelVel.scaleBy(1.0 / maxWheelSpeed);
// 		brWheelVel.scaleBy(1.0 / maxWheelSpeed);
// 		blWheelVel.scaleBy(1.0 / maxWheelSpeed);
// 		flWheelVel.scaleBy(1.0 / maxWheelSpeed);
// 	}

// 	// update steer motors
// 	Rotation2D frWheelYaw = Rotation2D(frWheelVel.getX(), frWheelVel.getY()).rotateBy(Rotation2D::fromDegrees(-90));
// 	Rotation2D brWheelYaw = Rotation2D(brWheelVel.getX(), brWheelVel.getY()).rotateBy(Rotation2D::fromDegrees(-90));
// 	Rotation2D blWheelYaw = Rotation2D(blWheelVel.getX(), blWheelVel.getY()).rotateBy(Rotation2D::fromDegrees(-90));
// 	Rotation2D flWheelYaw = Rotation2D(flWheelVel.getX(), flWheelVel.getY()).rotateBy(Rotation2D::fromDegrees(-90));

// 	m_pFRSteerMotorController->update(frWheelYaw.getDegrees(), 0, 0);
// 	m_pBRSteerMotorController->update(brWheelYaw.getDegrees(), 0, 0);
// 	m_pBLSteerMotorController->update(blWheelYaw.getDegrees(), 0, 0);
// 	m_pFLSteerMotorController->update(flWheelYaw.getDegrees(), 0, 0);

// 	// update drive motors
// 	m_pFRDriveMotorController->updateOpenLoopControl(frWheelVel.rotateBy(frWheelYaw.inverse()).getY());
// 	m_pBRDriveMotorController->updateOpenLoopControl(brWheelVel.rotateBy(brWheelYaw.inverse()).getY());
// 	m_pBLDriveMotorController->updateOpenLoopControl(blWheelVel.rotateBy(blWheelYaw.inverse()).getY());
// 	m_pFLDriveMotorController->updateOpenLoopControl(flWheelVel.rotateBy(flWheelYaw.inverse()).getY());
// }

// void SwerveDrivetrain::driveClosedLoopControl(
// 	double robotVelX,
// 	double robotVelY,
// 	double robotAccelX,
// 	double robotAccelY,
// 	double robotYawRate,
// 	double robotYawAccel) {

//     // convert robot vel to wheel vel
// 	Translation2D frWheelVel;
// 	Translation2D brWheelVel;
// 	Translation2D blWheelVel;
// 	Translation2D flWheelVel;
// 	m_kinematics.inverseKinematics(
// 		Translation2D(robotVelX, robotVelY),
// 		robotYawRate,
// 		frWheelVel,
// 		brWheelVel,
// 		blWheelVel,
// 		flWheelVel);

// 	// limit wheel vel
// 	double maxWheelSpeed = std::max(frWheelVel.norm(), std::max(brWheelVel.norm(), std::max(blWheelVel.norm(), flWheelVel.norm())));
// 	if(maxWheelSpeed > RobotParameters::k_maxSpeed) {
// 		frWheelVel.scaleBy(RobotParameters::k_maxSpeed / maxWheelSpeed);
// 		brWheelVel.scaleBy(RobotParameters::k_maxSpeed / maxWheelSpeed);
// 		blWheelVel.scaleBy(RobotParameters::k_maxSpeed / maxWheelSpeed);
// 		flWheelVel.scaleBy(RobotParameters::k_maxSpeed / maxWheelSpeed);
// 	}

//     // convert robot accel to wheel accel
// 	Translation2D frWheelAccel;
// 	Translation2D brWheelAccel;
// 	Translation2D blWheelAccel;
// 	Translation2D flWheelAccel;
// 	m_kinematics.inverseKinematics(
// 		Translation2D(robotAccelX, robotAccelY),
// 		robotYawAccel,
// 		frWheelAccel,
// 		brWheelAccel,
// 		blWheelAccel,
// 		flWheelAccel);

// 	// limit wheel accel
// 	double maxWheelAccel = std::max(frWheelAccel.norm(), std::max(brWheelAccel.norm(), std::max(blWheelAccel.norm(), flWheelAccel.norm())));
// 	if(maxWheelAccel > RobotParameters::k_maxAccel) {
// 		frWheelAccel.scaleBy(RobotParameters::k_maxAccel / maxWheelAccel);
// 		brWheelAccel.scaleBy(RobotParameters::k_maxAccel / maxWheelAccel);
// 		blWheelAccel.scaleBy(RobotParameters::k_maxAccel / maxWheelAccel);
// 		flWheelAccel.scaleBy(RobotParameters::k_maxAccel / maxWheelAccel);
// 	}

//     // update steer motors
// 	Rotation2D frWheelYaw = Rotation2D(frWheelVel.getX(), frWheelVel.getY()).rotateBy(Rotation2D::fromDegrees(-90));
// 	Rotation2D brWheelYaw = Rotation2D(brWheelVel.getX(), brWheelVel.getY()).rotateBy(Rotation2D::fromDegrees(-90));
// 	Rotation2D blWheelYaw = Rotation2D(blWheelVel.getX(), blWheelVel.getY()).rotateBy(Rotation2D::fromDegrees(-90));
// 	Rotation2D flWheelYaw = Rotation2D(flWheelVel.getX(), flWheelVel.getY()).rotateBy(Rotation2D::fromDegrees(-90));

// 	m_pFRSteerMotorController->update(frWheelYaw.getDegrees(), 0, 0);
// 	m_pBRSteerMotorController->update(brWheelYaw.getDegrees(), 0, 0);
// 	m_pBLSteerMotorController->update(blWheelYaw.getDegrees(), 0, 0);
// 	m_pFLSteerMotorController->update(flWheelYaw.getDegrees(), 0, 0);

//     // convert wheel vel from translational to rotational
//     double frWheelAngVel = frWheelVel.rotateBy(frWheelYaw.inverse()).getY() / RobotParameters::k_wheelRad * 180.0 / MATH_CONSTANTS_PI;
//     double brWheelAngVel = brWheelVel.rotateBy(brWheelYaw.inverse()).getY() / RobotParameters::k_wheelRad * 180.0 / MATH_CONSTANTS_PI;
//     double blWheelAngVel = blWheelVel.rotateBy(blWheelYaw.inverse()).getY() / RobotParameters::k_wheelRad * 180.0 / MATH_CONSTANTS_PI;
//     double flWheelAngVel = flWheelVel.rotateBy(flWheelYaw.inverse()).getY() / RobotParameters::k_wheelRad * 180.0 / MATH_CONSTANTS_PI;
//     double frWheelAngAccel = frWheelAccel.rotateBy(frWheelYaw.inverse()).getY() / RobotParameters::k_wheelRad * 180.0 / MATH_CONSTANTS_PI;
//     double brWheelAngAccel = brWheelAccel.rotateBy(brWheelYaw.inverse()).getY() / RobotParameters::k_wheelRad * 180.0 / MATH_CONSTANTS_PI;
//     double blWheelAngAccel = blWheelAccel.rotateBy(blWheelYaw.inverse()).getY() / RobotParameters::k_wheelRad * 180.0 / MATH_CONSTANTS_PI;
//     double flWheelAngAccel = flWheelAccel.rotateBy(flWheelYaw.inverse()).getY() / RobotParameters::k_wheelRad * 180.0 / MATH_CONSTANTS_PI;

//     // update drive motor
//     m_pFRDriveMotorController->updateClosedLoopControl(frWheelAngVel, frWheelAngAccel);
//     m_pBRDriveMotorController->updateClosedLoopControl(brWheelAngVel, brWheelAngAccel);
//     m_pBLDriveMotorController->updateClosedLoopControl(blWheelAngVel, blWheelAngAccel);
//     m_pFLDriveMotorController->updateClosedLoopControl(flWheelAngVel, flWheelAngAccel);
// }

// void SwerveDrivetrain::stop() {
//     driveOpenLoopControl(0, 0, 0);
// }

// void SwerveDrivetrain::setShiftState(bool isHighGear) {
// //	m_pShifter->Set(isHighGear);
// }

// bool SwerveDrivetrain::getShiftState() {
// //	bool isHighGear = m_pShifter->Get();
// 	bool isHighGear = false;

// 	// set appropriate motor controller gear ratio
// 	if(isHighGear) {
// 		m_pFRDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioHigh);
// 		m_pBRDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioHigh);
// 		m_pBLDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioHigh);
// 		m_pFLDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioHigh);
// 	}
// 	else {
// 		m_pFRDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
// 		m_pBRDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
// 		m_pBLDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
// 		m_pFLDriveMotorController->setTicksPerRev(RobotParameters::k_grayhillEncoderTicksPerRev * RobotParameters::k_driveMotorToEncoderGearRatioLow);
// 	}

// 	return isHighGear;
// }

// Pose2D SwerveDrivetrain::getPose() {
//     return m_swerveDrivePose.getPose();
// }

// PoseDot2D SwerveDrivetrain::getPoseDot() {
//     return m_swerveDrivePose.getPoseDot();
// }

// void SwerveDrivetrain::updatePose() {
//     // read front right wheel encoder
//     double oldFRWheelDist = m_frWheelDist;
//     m_frWheelDist = m_pFRDriveEncoder->getWheelDistance(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
//     SmartDashboard::PutNumber("frWheelDist", m_frWheelDist);
//     double deltaDistFRWheel = m_frWheelDist - oldFRWheelDist;
//     double frWheelVel = m_pFRDriveEncoder->getWheelVelocity(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
//     SmartDashboard::PutNumber("frWheelVel", frWheelVel);

// //    // check for wheel slip
// //    if(fabs(frWheelVel) > fabs(m_frWheelVelCmd * RobotParameters::k_wheelSlipNoiseRatio)) {
// //            // account for sample time and measurement noise
// //        deltaDistFRWheel = m_frWheelVelCmd * 1.0 / (double)RobotParameters::k_updateRate;
// //        frWheelVel = m_frWheelVelCmd;
// //    }

//     // read back right wheel encoder
//     double oldBRWheelDist = m_brWheelDist;
//     m_brWheelDist = m_pBRDriveEncoder->getWheelDistance(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
//     SmartDashboard::PutNumber("brWheelDist", m_brWheelDist);
//     double deltaDistBRWheel = m_brWheelDist - oldBRWheelDist;
//     double brWheelVel = m_pBRDriveEncoder->getWheelVelocity(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
//     SmartDashboard::PutNumber("brWheelVel", brWheelVel);

// //    // check for wheel slip
// //    if(fabs(brWheelVel) > fabs(m_brWheelVelCmd * RobotParameters::k_wheelSlipNoiseRatio)) {
// //            // account for sample time and measurement noise
// //        deltaDistBRWheel = m_brWheelVelCmd * 1.0 / (double)RobotParameters::k_updateRate;
// //        brWheelVel = m_brWheelVelCmd;
// //    }

//     // read back left wheel encoder
//     double oldBLWheelDist = m_blWheelDist;
//     m_blWheelDist = m_pBLDriveEncoder->getWheelDistance(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
//     SmartDashboard::PutNumber("blWheelDist", m_blWheelDist);
//     double deltaDistBLWheel = m_blWheelDist - oldBLWheelDist;
//     double blWheelVel = m_pBLDriveEncoder->getWheelVelocity(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
//     SmartDashboard::PutNumber("blWheelVel", blWheelVel);

// //    // check for wheel slip
// //    if(fabs(blWheelVel) > fabs(m_blWheelVelCmd * RobotParameters::k_wheelSlipNoiseRatio)) {
// //            // account for sample time and measurement noise
// //        deltaDistBLWheel = m_blWheelVelCmd * 1.0 / (double)RobotParameters::k_updateRate;
// //        blWheelVel = m_blWheelVelCmd;
// //    }

//     // read front right wheel encoder
//     double oldFLWheelDist = m_flWheelDist;
//     m_flWheelDist = m_pFLDriveEncoder->getWheelDistance(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
//     SmartDashboard::PutNumber("flWheelDist", m_flWheelDist);
//     double deltaDistFLWheel = m_flWheelDist - oldFLWheelDist;
//     double flWheelVel = m_pFLDriveEncoder->getWheelVelocity(RobotParameters::k_wheelRad, RobotParameters::k_driveEncoderToWheelGearRatio);
//     SmartDashboard::PutNumber("flWheelVel", flWheelVel);

// //    // check for wheel slip
// //    if(fabs(flWheelVel) > fabs(m_flWheelVelCmd * RobotParameters::k_wheelSlipNoiseRatio)) {
// //            // account for sample time and measurement noise
// //        deltaDistFLWheel = m_flWheelVelCmd * 1.0 / (double)RobotParameters::k_updateRate;
// //        flWheelVel = m_flWheelVelCmd;
// //    }

//     // read IMU
//     double oldGyroYaw = m_gyroYaw;
//     m_gyroYaw = -m_pChassisIMU->GetYaw();
//     double deltaGyroYaw = m_gyroYaw - oldGyroYaw;
//     double gyroYawRate = -m_pChassisIMU->GetRate() * 180.0 / MATH_CONSTANTS_PI;

//     // update pose
//     m_swerveDrivePose.update(
//     	deltaDistFRWheel,
// 		deltaDistBRWheel,
// 		deltaDistBLWheel,
// 		deltaDistFLWheel,
// 		Rotation2D::fromDegrees(m_pFRSteerEncoder->getAngle()),
// 		Rotation2D::fromDegrees(m_pBRSteerEncoder->getAngle()),
// 		Rotation2D::fromDegrees(m_pBLSteerEncoder->getAngle()),
// 		Rotation2D::fromDegrees(m_pFLSteerEncoder->getAngle()),
// 		deltaGyroYaw,
// 		frWheelVel,
// 		brWheelVel,
// 		blWheelVel,
// 		flWheelVel,
// 		gyroYawRate);

//     SmartDashboard::PutNumber("x", m_swerveDrivePose.getPose().getTranslation().getX());
//     SmartDashboard::PutNumber("y", m_swerveDrivePose.getPose().getTranslation().getY());
//     SmartDashboard::PutNumber("yaw", m_swerveDrivePose.getPose().getRotation().getDegrees());

//     SmartDashboard::PutNumber("xVel", m_swerveDrivePose.getPoseDot().getXVel());
//     SmartDashboard::PutNumber("yVel", m_swerveDrivePose.getPoseDot().getYVel());
// 	SmartDashboard::PutNumber("yawRate", m_swerveDrivePose.getPoseDot().getYawRate());
// }

// void SwerveDrivetrain::resetPose(const Pose2D &pose, const PoseDot2D &poseDot) {
// 	m_swerveDrivePose.reset(pose, poseDot);
// }

// void SwerveDrivetrain::zeroDriveEncoders() {
// 	m_pFRDriveEncoder->zero();
// 	m_pBRDriveEncoder->zero();
// 	m_pBLDriveEncoder->zero();
// 	m_pFLDriveEncoder->zero();
// }

// void SwerveDrivetrain::zeroSteerEncoders() {
// 	m_pFRSteerEncoder->zero();
// 	m_pBRSteerEncoder->zero();
// 	m_pBLSteerEncoder->zero();
// 	m_pFLSteerEncoder->zero();
// }

// void SwerveDrivetrain::zeroGyroYaw() {
// 	m_pChassisIMU->ZeroYaw();
// }
