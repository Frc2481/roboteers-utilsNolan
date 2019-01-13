#ifndef POSE_2D_H
#define POSE_2D_H

#include "Rotation2D.h"
#include "Translation2D.h"

class Pose2D {
public:
    Pose2D();
    Pose2D(const Translation2D &translation, const Rotation2D &rotation);
    Pose2D(const Pose2D &other);
    static Pose2D fromTranslation(const Translation2D &translation);
    static Pose2D fromRotation(const Rotation2D &rotation);
    ~Pose2D();

    Pose2D& operator=(const Pose2D &other);

    Translation2D getTranslation() const;
    Rotation2D getRotation() const;
    
    void setTranslation(const Translation2D &translation);
    void setRotation(const Rotation2D &rotation);

private:
    Translation2D m_translation;
    Rotation2D m_rotation;
};

#endif // POSE_2D_H
