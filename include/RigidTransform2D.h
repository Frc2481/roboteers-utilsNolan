//////////////////////////////////////////////////////////////////////
// 2D rigid body transformations
//////////////////////////////////////////////////////////////////////
#ifndef RIGID_TRANSFORM_2D_H
#define RIGID_TRANSFORM_2D_H

#include "Rotation2D.h"
#include "Translation2D.h"

class RigidTransform2D {
public:
    RigidTransform2D();
    RigidTransform2D(const Translation2D &translation, const Rotation2D &rotation);
    RigidTransform2D(const RigidTransform2D &other);
    static RigidTransform2D fromTranslation(const Translation2D &translation);
    static RigidTransform2D fromRotation(const Rotation2D &rotation);
    ~RigidTransform2D();

    RigidTransform2D& operator=(const RigidTransform2D &other);

    Translation2D getTranslation() const;
    Rotation2D getRotation() const;
    
    void setTranslation(const Translation2D &translation);
    void setRotation(const Rotation2D &rotation);

private:
    Translation2D m_translation;
    Rotation2D m_rotation;
};

#endif // RIGID_TRANSFORM_2D_H
