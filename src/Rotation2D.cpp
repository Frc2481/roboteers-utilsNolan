#include "Rotation2D.h"
#include <cmath>
#include <limits>

Rotation2D::Rotation2D()
    : m_cos(1),
    m_sin(0) {
}

Rotation2D::Rotation2D(const double &cos, const double &sin)
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

Rotation2D Rotation2D::fromRadians(const double &angle) {
    return Rotation2D(cos(angle), sin(angle));
}

Rotation2D Rotation2D::fromDegrees(const double &angle) {
    return fromRadians(angle * M_PI / 180.0);
}

Rotation2D& Rotation2D::operator=(const Rotation2D &rhs) {
	m_cos = rhs.m_cos;
	m_sin = rhs.m_sin;

	return *this;
}

Rotation2D& Rotation2D::operator+=(const Rotation2D &rhs) {
    this->m_cos = this->rotateBy(rhs).m_cos;
    this->m_sin = this->rotateBy(rhs).m_sin;

    return *this;
}

Rotation2D& Rotation2D::operator-=(const Rotation2D &rhs) {
    this->m_cos = this->rotateBy(rhs.inverse()).m_cos;
    this->m_sin = this->rotateBy(rhs.inverse()).m_sin;

    return *this;
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
    return getRadians() * 180.0 / M_PI;
}

void Rotation2D::setRadians(const double &angle) {
    m_cos = cos(angle);
    m_sin = sin(angle);
}

void Rotation2D::setDegrees(const double &angle) {
    m_cos = cos(angle * M_PI / 180.0);
    m_sin = sin(angle * M_PI / 180.0);
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

const Rotation2D Rotation2D::operator+(const Rotation2D &other) {
    return (*this).rotateBy(other);
}

const Rotation2D Rotation2D::operator-(const Rotation2D &other) {
    return (*this).rotateBy(other.inverse());
}
