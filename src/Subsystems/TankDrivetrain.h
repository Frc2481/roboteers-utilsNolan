#ifndef TANK_DRIVETRAIN_H
#define TANK_DRIVETRAIN_H

#include <WPILib.h>
#include "ctre/Phoenix.h"
#include "Components/GrayhillEncoder.h"
#include "Utils/Pose2D.h"
#include "Utils/TankDrivePose.h"
#include "Utils/MotorVelocityController.h"

//class AHRS;

// +x = robot right
// +y = robot forward
// +yaw = CCW, zero is robot forward

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

class TankDrivetrain : public Subsystem {
public:
    TankDrivetrain();
    ~TankDrivetrain();
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
    //////////////////////////////////////////////////////////////////////
    void driveClosedLoopControl(
        double robotVel,
        double robotYawRate,
        double robotAccel);
    
    void stop();

    void setShiftState(bool isHighGear);
    bool getShiftState();

    Pose2D getPose();
    Pose2D getPoseDot();
    void updatePose();
    void resetPose(const Pose2D &pose, const Pose2D &poseDot);

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
//    AHRS* m_pChassisIMU;
    TankDriveKinematics m_kinematics;
    TankDrivePose m_tankDrivePose;
    double m_leftWheelDist;
    double m_rightWheelDist;
    double m_leftWheelVelCmd;
    double m_rightWheelVelCmd;
    double m_gyroYaw;
};

#endif // TANK_DRIVETRAIN_H
