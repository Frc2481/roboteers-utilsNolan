#include "Translation2D.h"
#include <cmath>

Translation2D::Translation2D()
	: m_x(0),
    m_y(0) {
}

Translation2D::Translation2D(double x, double y)
	: m_x(x),
    m_y(y) {
}

Translation2D::Translation2D(const Translation2D &other)
	: m_x(other.m_x),
    m_y(other.m_y) {
}

Translation2D::~Translation2D() {
}

Translation2D& Translation2D::operator=(const Translation2D &rhs) {
    m_x = rhs.m_x;
	m_y = rhs.m_y;

	return *this;
}

Translation2D& Translation2D::operator+=(const Translation2D &rhs) {
    *this = *this + rhs;

	return *this;
}

Translation2D& Translation2D::operator-=(const Translation2D &rhs) {
    *this = *this - rhs;

    return *this;
}

Translation2D Translation2D::operator+(const Translation2D &other) const {
    return (*this).translateBy(other);
}

Translation2D Translation2D::operator-(const Translation2D &other) const {
    return (*this).translateBy(other.inverse());
}

double Translation2D::getX() const {
	return m_x;
}

double Translation2D::getY() const {
	return m_y;
}

void Translation2D::setX(double x) {
	m_x = x;
}

void Translation2D::setY(double y)
{
	m_y = y;
}

double Translation2D::norm() const {
	return sqrt(m_x * m_x + m_y * m_y);
}

double Translation2D::dot(const Translation2D &other) const {
    return m_x * other.getX() + m_y * other.getY();
}

double Translation2D::cross(const Translation2D &other) const {
    return m_x * other.getY() - m_y * other.getX();
}

Translation2D Translation2D::inverse() const {
	return Translation2D(-m_x, -m_y);
}

Translation2D Translation2D::translateBy(const Translation2D &other) const {
	return Translation2D(m_x + other.getX(), m_y + other.getY());
}

Translation2D Translation2D::rotateBy(const Rotation2D &rotation) const {
	return Translation2D(m_x * rotation.getCos() - m_y * rotation.getSin(), m_x * rotation.getSin() + m_y * rotation.getCos());
}

Translation2D Translation2D::scaleBy(double scalar) const {
	return Translation2D(m_x * scalar, m_y * scalar);
}
