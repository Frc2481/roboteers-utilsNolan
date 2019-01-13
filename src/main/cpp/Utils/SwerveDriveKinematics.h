#ifndef SWERVE_DRIVE_KINEMATICS_H
#define SWERVE_DRIVE_KINEMATICS_H

#include "Translation2D.h"

// +x = robot right
// +y = robot forward
// +yaw = CCW, zero is robot forward

class SwerveDriveKinematics {
public:
	SwerveDriveKinematics(
		Translation2D frLeverArm,
		Translation2D brLeverArm,
		Translation2D blLeverArm,
		Translation2D flLeverArm);
    ~SwerveDriveKinematics();

    //////////////////////////////////////////////////////////////////////
    // @brief calculate robot velocity and yaw rate from wheel velocities
    // @param frWheelVel - front right wheel velocity (in/s)
    // @param brWheelVel - back right wheel velocity (in/s)
    // @param blWheelVel - back left wheel velocity (in/s)
    // @param flWheelVel - front left wheel velocity (in/s)
    // @param robotVel - robot velocity (in/s)
    // @param robotYawRate - robot yaw rate (deg/s)
    //////////////////////////////////////////////////////////////////////
    void forwardKinematics(
    	Translation2D frWheelVel,
		Translation2D brWheelVel,
		Translation2D blWheelVel,
		Translation2D flWheelVel,
		Translation2D &robotVel,
		double &robotYawRate);

    //////////////////////////////////////////////////////////////////////
    // @brief calculate wheel velocities from robot velocity and yaw rate
    // @param robotVel - robot velocity (in/s)
    // @param robotYawRate - robot yaw rate (deg/s)
    // @param frWheelVel - front right wheel velocity (in/s)
	// @param brWheelVel - back right wheel velocity (in/s)
	// @param blWheelVel - back left wheel velocity (in/s)
	// @param flWheelVel - front left wheel velocity (in/s)
    //////////////////////////////////////////////////////////////////////
    void inverseKinematics(
		Translation2D robotVel,
		double robotYawRate,
		Translation2D &frWheelVel,
		Translation2D &brWheelVel,
		Translation2D &blWheelVel,
		Translation2D &flWheelVel);

private:
    Translation2D m_frLeverArm;
    Translation2D m_brLeverArm;
    Translation2D m_blLeverArm;
    Translation2D m_flLeverArm;
};

#endif // SWERVE_DRIVE_KINEMATICS_H
