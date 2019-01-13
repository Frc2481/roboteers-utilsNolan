#include "Rotation2D.h"
#include <cmath>
#include <limits>
#include "MathConstants.h"

Rotation2D::Rotation2D()
    : m_cos(1),
    m_sin(0) {
}

Rotation2D::Rotation2D(double cos, double sin)
    : m_cos(cos),
    m_sin(sin) {

    // ensure that cos and sin are normalized to unit circle
    normalize();
}

Rotation2D::Rotation2D(const Rotation2D &other)
    : m_cos(other.m_cos),
    m_sin(other.m_sin) {
}

Rotation2D::~Rotation2D() {
}

Rotation2D Rotation2D::fromRadians(double angle) {
    return Rotation2D(cos(angle), sin(angle));
}

Rotation2D Rotation2D::fromDegrees(double angle) {
    return fromRadians(angle * MATH_CONSTANTS_PI / 180.0);
}

Rotation2D& Rotation2D::operator=(const Rotation2D &rhs) {
	m_cos = rhs.m_cos;
	m_sin = rhs.m_sin;

	return *this;
}

Rotation2D& Rotation2D::operator+=(const Rotation2D &rhs) {
    *this = *this + rhs;

    return *this;
}

Rotation2D& Rotation2D::operator-=(const Rotation2D &rhs) {
    *this = *this - rhs;

    return *this;
}

Rotation2D Rotation2D::operator+(const Rotation2D &other) const {
    return (*this).rotateBy(other);
}

Rotation2D Rotation2D::operator-(const Rotation2D &other) const {
    return (*this).rotateBy(other.inverse());
}

double Rotation2D::getCos() const {
    return m_cos;
}

double Rotation2D::getSin() const {
    return m_sin;
}

double Rotation2D::getTan() const {
    if (fabs(m_cos) < 1E-9) {
        if (fabs(m_sin) >= 0.0) {
            return std::numeric_limits<double>::infinity();
        }
        else {
            return -std::numeric_limits<double>::infinity();
        }
    }

    return m_sin / m_cos;
}

double Rotation2D::getRadians() const {
    return atan2(m_sin, m_cos);
}

double Rotation2D::getDegrees() const {
    return getRadians() * 180.0 / MATH_CONSTANTS_PI;
}

void Rotation2D::setRadians(double angle) {
    m_cos = cos(angle);
    m_sin = sin(angle);
}

void Rotation2D::setDegrees(double angle) {
    m_cos = cos(angle * MATH_CONSTANTS_PI / 180.0);
    m_sin = sin(angle * MATH_CONSTANTS_PI / 180.0);
}

Rotation2D Rotation2D::rotateBy(const Rotation2D &other) const {
    double newCos = m_cos * other.m_cos - m_sin * other.m_sin;
    double newSin = m_cos * other.m_sin + m_sin * other.m_cos;

    return Rotation2D(newCos, newSin);
}

Rotation2D Rotation2D::inverse() const {
    return Rotation2D(m_cos, -m_sin);
}

void Rotation2D::normalize() {
    double norm = hypot(m_cos, m_sin);
    if (norm > 1E-9) {
        m_sin /= norm;
        m_cos /= norm;
    }
    else {
        m_sin = 0;
        m_cos = 1;
    }
}
