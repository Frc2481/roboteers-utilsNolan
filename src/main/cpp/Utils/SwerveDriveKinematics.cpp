#include "SwerveDriveKinematics.h"
#include "MathConstants.h"

SwerveDriveKinematics::SwerveDriveKinematics(
	Translation2D frLeverArm,
	Translation2D brLeverArm,
	Translation2D blLeverArm,
	Translation2D flLeverArm)

    : m_frLeverArm(frLeverArm),
	  m_brLeverArm(brLeverArm),
	  m_blLeverArm(blLeverArm),
	  m_flLeverArm(flLeverArm) {
}

SwerveDriveKinematics::~SwerveDriveKinematics() {
}

void SwerveDriveKinematics::forwardKinematics(
	Translation2D frWheelVel,
	Translation2D brWheelVel,
	Translation2D blWheelVel,
	Translation2D flWheelVel,
	Translation2D &robotVel,
	double &robotYawRate) {

	double robotYawRate1 = -(frWheelVel.getX() - blWheelVel.getX()) / (m_frLeverArm.getY() - m_blLeverArm.getY());
	double robotYawRate2 = (frWheelVel.getY() - blWheelVel.getY()) / (m_frLeverArm.getX() - m_blLeverArm.getX());
	double robotYawRate3 = -(flWheelVel.getX() - brWheelVel.getX()) / (m_flLeverArm.getY() - m_brLeverArm.getY());
	double robotYawRate4 = (flWheelVel.getY() - brWheelVel.getY()) / (m_flLeverArm.getX() - m_brLeverArm.getX());
	robotYawRate = (robotYawRate1 + robotYawRate2 + robotYawRate3 + robotYawRate4) / 4.0;

	Translation2D robotVel1 = frWheelVel - Translation2D(-robotYawRate * m_frLeverArm.getY(), robotYawRate * m_frLeverArm.getX());
	Translation2D robotVel2 = brWheelVel - Translation2D(-robotYawRate * m_brLeverArm.getY(), robotYawRate * m_brLeverArm.getX());
	Translation2D robotVel3 = blWheelVel - Translation2D(-robotYawRate * m_blLeverArm.getY(), robotYawRate * m_blLeverArm.getX());
	Translation2D robotVel4 = flWheelVel - Translation2D(-robotYawRate * m_flLeverArm.getY(), robotYawRate * m_flLeverArm.getX());
	robotVel = (robotVel1 + robotVel2 + robotVel3 + robotVel4).scaleBy(1.0 / 4.0);

	robotYawRate *= 180.0 / MATH_CONSTANTS_PI; // convert to deg/s
}

void SwerveDriveKinematics::inverseKinematics(
	Translation2D robotVel,
	double robotYawRate,
	Translation2D &frWheelVel,
	Translation2D &brWheelVel,
	Translation2D &blWheelVel,
	Translation2D &flWheelVel) {

	robotYawRate *= MATH_CONSTANTS_PI / 180.0; // convert to rad/s
	frWheelVel = Translation2D(-robotYawRate * m_frLeverArm.getY(), robotYawRate * m_frLeverArm.getX()) + robotVel;
	brWheelVel = Translation2D(-robotYawRate * m_brLeverArm.getY(), robotYawRate * m_brLeverArm.getX()) + robotVel;
	blWheelVel = Translation2D(-robotYawRate * m_blLeverArm.getY(), robotYawRate * m_blLeverArm.getX()) + robotVel;
	flWheelVel = Translation2D(-robotYawRate * m_flLeverArm.getY(), robotYawRate * m_flLeverArm.getX()) + robotVel;
}
