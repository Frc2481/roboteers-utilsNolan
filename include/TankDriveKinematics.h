#ifndef TANK_DRIVE_KINEMATICS_H
#define TANK_DRIVE_KINEMATICS_H

#include "Translation2D.h"

// +x = robot right
// +y = robot forward
// +yaw = CCW, zero is robot forward

class TankDriveKinematics {
public:
    TankDriveKinematics();
    ~TankDriveKinematics();

    void setWheelTrack(const double &wheelTrack);

    //////////////////////////////////////////////////////////////////////
    // @brief calculate forward robot velocity and yaw rate from forward
    //        wheel velocities
    // @param leftWheelVel - left wheel velocity (in/s)
    // @param rightWheelVel - right wheel velocity (in/s)
    // @param robotVel - robot velocity (in/s)
    // @param robotYawRate - robot yaw rate (deg/s)
    //////////////////////////////////////////////////////////////////////
    void forwardKinematics(
        const double &leftWheelVel,
        const double &rightWheelVel,
        double &robotVel,
        double &robotYawRate);

    //////////////////////////////////////////////////////////////////////
    // @brief calculate forward wheel velocities from forward robot velocity
    //        and yaw rate
    // @param robotVel - robot velocity (in/s)
    // @param robotYawRate - robot yaw rate (deg/s)
    // @param leftWheelVel - left wheel velocity (in/s)
    // @param rightWheelVel - right wheel velocity (in/s)
    //////////////////////////////////////////////////////////////////////
    void inverseKinematics(
        const double &robotVel,
        const double &robotYawRate,
        double &leftWheelVel,
        double &rightWheelVel);

private:
    double m_wheelTrack;
};

#endif // TANK_DRIVE_KINEMATICS_H
