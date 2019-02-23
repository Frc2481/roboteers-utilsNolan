#include "Utils/PoseDot2D.h"

PoseDot2D::PoseDot2D()
    : m_xVel(0),
	m_yVel(0),
	m_yawRate(0) {
}

PoseDot2D::PoseDot2D(double xVel, double yVel, double yawRateDegPerSec)
    : m_xVel(xVel),
    m_yVel(yVel),
	m_yawRate(yawRateDegPerSec) {
}

PoseDot2D::PoseDot2D(const PoseDot2D &other)
    : m_xVel(other.m_xVel),
	m_yVel(other.m_yVel),
	m_yawRate(other.m_yawRate) {
}

PoseDot2D::~PoseDot2D() {
}

PoseDot2D& PoseDot2D::operator=(const PoseDot2D &rhs) {
	m_xVel = rhs.m_xVel;
	m_yVel = rhs.m_yVel;
	m_yawRate = rhs.m_yawRate;

	return *this;
}

double PoseDot2D::getXVel() {
	return m_xVel;
}

double PoseDot2D::getYVel() {
	return m_yVel;
}

double PoseDot2D::getYawRate() {
	return m_yawRate;
}
