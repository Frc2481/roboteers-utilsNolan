#include "RigidTransform2D.h"

RigidTransform2D::RigidTransform2D()
    : m_translation(Translation2D()),
    m_rotation(Rotation2D()) {
}

RigidTransform2D::RigidTransform2D(const Translation2D &translation, const Rotation2D &rotation)
    : m_translation(translation),
    m_rotation(rotation) {
}

RigidTransform2D::RigidTransform2D(const RigidTransform2D &other)
    : m_translation(other.m_translation),
    m_rotation(other.m_rotation) {
}

RigidTransform2D RigidTransform2D::fromTranslation(const Translation2D &translation) {
    return RigidTransform2D(translation, Rotation2D());
}

RigidTransform2D RigidTransform2D::fromRotation(const Rotation2D &rotation) {
    return RigidTransform2D(Translation2D(), rotation);
}

RigidTransform2D::~RigidTransform2D() {
}

RigidTransform2D& RigidTransform2D::operator=(const RigidTransform2D &rhs) {
    m_translation = rhs.m_translation;
	m_rotation = rhs.m_rotation;

	return *this;
}

RigidTransform2D RigidTransform2D::transformBy(const RigidTransform2D &other) const {
    Translation2D newTranslation = m_translation.translateBy(other.m_translation.rotateBy(m_rotation));
    Rotation2D newRotation = m_rotation.rotateBy(other.m_rotation);

    return RigidTransform2D(newTranslation, newRotation);
}

Translation2D RigidTransform2D::getTranslation() const {
    return m_translation;
}

Rotation2D RigidTransform2D::getRotation() const {
    return m_rotation;
}

void RigidTransform2D::setTranslation(const Translation2D &translation) {
    m_translation = translation;
}

void RigidTransform2D::setRotation(const Rotation2D &rotation) {
    m_rotation = rotation;
}

RigidTransform2D RigidTransform2D::inverse() const {
    Rotation2D newRotation = m_rotation.inverse();
    Translation2D newTranslation = m_translation.inverse().rotateBy(newRotation);

    return RigidTransform2D(newTranslation, newRotation);
}
