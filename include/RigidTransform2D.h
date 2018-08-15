//////////////////////////////////////////////////////////////////////
// 2D rigid body transformations
//////////////////////////////////////////////////////////////////////
#pragma once
#include "Rotation2D.h"
#include "Translation2D.h"

class RigidTransform2D {
public:
    RigidTransform2D();
    RigidTransform2D(const Translation2D &translation, const Rotation2D &rotation);
    RigidTransform2D(const RigidTransform2D &other);
    RigidTransform2D fromTranslation(const Translation2D &translation);
    RigidTransform2D fromRotation(const Rotation2D &rotation);
    ~RigidTransform2D();

    RigidTransform2D& operator=(const RigidTransform2D &other);

    //////////////////////////////////////////////////////////////////////
    // @brief transform transform by other transform
    //////////////////////////////////////////////////////////////////////
    RigidTransform2D transformBy(const RigidTransform2D &other);

    Translation2D getTranslation();
    Rotation2D getRotation();
    void setTranslation(const Translation2D &translation);
    void setRotation(const Rotation2D &rotation);

    //////////////////////////////////////////////////////////////////////
    // @brief inverse of transformation
    //////////////////////////////////////////////////////////////////////
    RigidTransform2D inverse();

private:
    Translation2D m_translation;
    Rotation2D m_rotation;
};
