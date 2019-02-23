#include "Utils/Pose2D.h"

Pose2D::Pose2D()
    : m_translation(Translation2D()),
    m_rotation(Rotation2D()) {
}

Pose2D::Pose2D(const Translation2D &translation, const Rotation2D &rotation)
    : m_translation(translation),
    m_rotation(rotation) {
}

Pose2D::Pose2D(const Pose2D &other)
    : m_translation(other.m_translation),
    m_rotation(other.m_rotation) {
}

Pose2D Pose2D::fromTranslation(const Translation2D &translation) {
    return Pose2D(translation, Rotation2D());
}

Pose2D Pose2D::fromRotation(const Rotation2D &rotation) {
    return Pose2D(Translation2D(), rotation);
}

Pose2D::~Pose2D() {
}

Pose2D& Pose2D::operator=(const Pose2D &rhs) {
    m_translation = rhs.m_translation;
	m_rotation = rhs.m_rotation;

	return *this;
}

Translation2D Pose2D::getTranslation() const {
    return m_translation;
}

Rotation2D Pose2D::getRotation() const {
    return m_rotation;
}

void Pose2D::setTranslation(const Translation2D &translation) {
    m_translation = translation;
}

void Pose2D::setRotation(const Rotation2D &rotation) {
    m_rotation = rotation;
}
