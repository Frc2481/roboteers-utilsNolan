// #ifndef SWERVE_DRIVETRAIN_H
// #define SWERVE_DRIVETRAIN_H

// #include <frc/WPILib.h>
// #include "ctre/Phoenix.h"
// #include "AHRS.h"
// #include "../Components/GrayhillEncoder.h"
// #include "../Components/CTREMagEncoder.h"
// #include "../Utils/Pose2D.h"
// #include "../Utils/PoseDot2D.h"
// #include "../Utils/SwerveDrivePose.h"
// #include "../Utils/MotorPositionController.h"
// #include "../Utils/MotorVelocityController.h"

// // +x = robot right
// // +y = robot forward
// // +yaw = CCW, zero is robot forward

// class SwerveDrivetrain : public frc::Subsystem {
// public:
// 	SwerveDrivetrain();
//     // ~SwerveDrivetrain();
//     virtual void InitDefaultCommand();
//     virtual void Periodic();

//     //////////////////////////////////////////////////////////////////////
//     // @brief command robot to drive using open loop control
//     // @param percentVelX - velocity x percent (-1 to 1)
//     // @param percentVelY - velocity y percent (-1 to 1)
//     // @param percentYawRate - yaw rate percent (-1 to 1)
//     //////////////////////////////////////////////////////////////////////
//     void driveOpenLoopControl(double percentVelX, double percentVelY, double percentYawRate);

//     //////////////////////////////////////////////////////////////////////
//     // @brief command robot to drive using closed loop control
//     // @param robotVel - forward velocity of robot (in/s)
//     // @param robotYawRate - yaw rate of robot (deg/s)
//     // @param robotAccel - forward acceleration of robot (in/s^2)
//     // @param robotYawAccel - yaw accel of robot (deg/s^2)
//     //////////////////////////////////////////////////////////////////////
//     void driveClosedLoopControl(
//         double robotVelX,
// 		double robotVelY,
// 		double robotAccelX,
// 		double robotAccelY,
//         double robotYawRate,
// 		double robotYawAccel);
    
//     void stop();

//     void setShiftState(bool isHighGear);
//     bool getShiftState();

//     Pose2D getPose();
//     PoseDot2D getPoseDot();
//     void updatePose();
//     void resetPose(const Pose2D &pose, const PoseDot2D &poseDot);
//     void zeroDriveEncoders();
//     void zeroSteerEncoders();
//     void zeroGyroYaw();

// private:
//     TalonSRX* m_pFRDriveMotor;
//     TalonSRX* m_pBRDriveMotor;
//     TalonSRX* m_pBLDriveMotor;
//     TalonSRX* m_pFLDriveMotor;
//     MotorVelocityController* m_pFRDriveMotorController;
//     MotorVelocityController* m_pBRDriveMotorController;
//     MotorVelocityController* m_pBLDriveMotorController;
//     MotorVelocityController* m_pFLDriveMotorController;
//     GrayhillEncoder* m_pFRDriveEncoder;
//     GrayhillEncoder* m_pBRDriveEncoder;
//     GrayhillEncoder* m_pBLDriveEncoder;
//     GrayhillEncoder* m_pFLDriveEncoder;
//     TalonSRX* m_pFRSteerMotor;
// 	TalonSRX* m_pBRSteerMotor;
// 	TalonSRX* m_pBLSteerMotor;
// 	TalonSRX* m_pFLSteerMotor;
// 	MotorPositionController* m_pFRSteerMotorController;
// 	MotorPositionController* m_pBRSteerMotorController;
// 	MotorPositionController* m_pBLSteerMotorController;
// 	MotorPositionController* m_pFLSteerMotorController;
// 	CTREMagEncoder* m_pFRSteerEncoder;
// 	CTREMagEncoder* m_pBRSteerEncoder;
// 	CTREMagEncoder* m_pBLSteerEncoder;
// 	CTREMagEncoder* m_pFLSteerEncoder;
//     Solenoid* m_pShifter;
//     AHRS* m_pChassisIMU;
//     SwerveDriveKinematics m_kinematics;
//     SwerveDrivePose m_swerveDrivePose;
//     double m_frWheelDist;
//     double m_brWheelDist;
//     double m_blWheelDist;
//     double m_flWheelDist;
//     double m_gyroYaw;
// };

// #endif // SWERVE_DRIVETRAIN_H
