#ifndef TANK_DRIVETRAIN_H
#define TANK_DRIVETRAIN_H

#include "WPILib.h"
#include "Commands/Subsystem.h"
#include "Utils/TankDriveKinematics.h"
#include "Utils/MotorVelocityController.h"
#include "ctre/Phoenix.h"
#include "Utils/GrayhillEncoder.h"

#define WHEEL_SLIP_NOISE_RATIO 1.2 // wheel encoder noise ratio used to detect wheel slip

// +x = robot right
// +y = robot forward
// +yaw = CCW, zero is robot forward

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

class TankDrivetrain : public Subsystem {
public:
    void TankDrivetrain();
    void ~TankDrivetrain();
    virtual void InitDefaultCommand();
    virtual void Periodic();

    //////////////////////////////////////////////////////////////////////
    // @brief command robot to drive using open loop control
    // @param percentLeftDrive - left drive percent (-1 to 1)
    // @param percentRightDrive - right drive percent (-1 to 1)
    //////////////////////////////////////////////////////////////////////
    void driveOpenLoopControl(double &percentLeftDrive, double &percentRightDrive)

    //////////////////////////////////////////////////////////////////////
    // @brief command robot to drive using closed loop control
    // @param robotVel - forward velocity of robot (in/s)
    // @param robotYawRate - yaw rate of robot (deg/s)
    // @param robotAccel - forward acceleration of robot (in/s^2)
    //////////////////////////////////////////////////////////////////////
    void driveClosedLoopControl(
        const double &robotVel,
        const double &robotYawRate,
        const double &robotAccel);
    
    void stop();

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
    AHRS* m_pChassisIMU;
    double m_wheelRad; // wheel radius (in)
    double m_wheelTrack; // wheel track (in)
    TankDriveKinematics m_kinematics;
    double m_maxSpeed; // max speed (in/s)
    double m_maxAccel; // max accel (in/s^2)
    double m_maxDeccel; // max deccel (in/s^2)
    double m_maxCentripAccel; // max centripetal accel (in/s^2)
    double m_updateRate; // update rate (Hz)
    TankDrivePose m_tankDrivePose;
    double m_driveGearRatio;
    double m_leftWheelDist;
    double m_rightWheelDist;
    double m_leftWheelVelCmd;
    double m_rightWheelVelCmd;
    double m_gyroYaw;
};

#endif // TANK_DRIVETRAIN_H
