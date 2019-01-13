#ifndef TANK_DRIVETRAIN_H
#define TANK_DRIVETRAIN_H

#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include "AHRS.h"
#include "../Components/GrayhillEncoder.h"
#include "../Utils/Pose2D.h"
#include "../Utils/PoseDot2D.h"
#include "../Utils/TankDrivePose.h"
#include "../Utils/MotorVelocityController.h"

// +x = robot right
// +y = robot forward
// +yaw = CCW, zero is robot forward

class TankDrivetrain : public frc::Subsystem {
public:
    TankDrivetrain();
    // ~TankDrivetrain();
    virtual void InitDefaultCommand();
    virtual void Periodic();

    //////////////////////////////////////////////////////////////////////
    // @brief command robot to drive using open loop control
    // @param percentLeftDrive - left drive percent (-1 to 1)
    // @param percentRightDrive - right drive percent (-1 to 1)
    //////////////////////////////////////////////////////////////////////
    void driveOpenLoopControl(double percentLeftDrive, double percentRightDrive);

    //////////////////////////////////////////////////////////////////////
    // @brief command robot to drive using closed loop control
    // @param robotVel - forward velocity of robot (in/s)
    // @param robotYawRate - yaw rate of robot (deg/s)
    // @param robotAccel - forward acceleration of robot (in/s^2)
    // @param robotYawAccel - yaw accel of robot (deg/s^2)
    //////////////////////////////////////////////////////////////////////
    void driveClosedLoopControl(
        double robotVel,
        double robotYawRate,
        double robotAccel,
		double robotYawAccel);
    
    void stop();

    void setShiftState(bool isHighGear);
    bool getShiftState();

    Pose2D getPose();
    PoseDot2D getPoseDot();
    void updatePose();
    void resetPose(const Pose2D &pose, const PoseDot2D &poseDot);
    void zeroDriveEncoders();
    void zeroGyroYaw();

private:
    TalonSRX* m_pLeftDriveMotor;
    TalonSRX* m_pRightDriveMotor;
    MotorVelocityController* m_pLeftDriveMotorController;
    MotorVelocityController* m_pRightDriveMotorController;
    GrayhillEncoder* m_pLeftDriveEncoder;
    GrayhillEncoder* m_pRightDriveEncoder;
    TalonSRX* m_pLeftDriveMotorSlave;
    TalonSRX* m_pRightDriveMotorSlave;
    Solenoid* m_pShifter;
    AHRS* m_pChassisIMU;
    TankDriveKinematics m_kinematics;
    TankDrivePose m_tankDrivePose;
    double m_leftWheelDist;
    double m_rightWheelDist;
    double m_gyroYaw;
};

#endif // TANK_DRIVETRAIN_H
