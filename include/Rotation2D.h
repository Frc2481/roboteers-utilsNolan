//////////////////////////////////////////////////////////////////////
// 2D rigid body rotations
//////////////////////////////////////////////////////////////////////
#pragma once

class Rotation2D {
public:
    Rotation2D();
    Rotation2D(const double &cos, const double &sin);
    Rotation2D(const Rotation2D &other);
    ~Rotation2D();
    static Rotation2D fromRadians(const double &angle);
    static Rotation2D fromDegrees(const double &angle);

    Rotation2D& operator=(const Rotation2D &rhs);
    Rotation2D& operator+=(const Rotation2D &rhs);
    Rotation2D& operator-=(const Rotation2D &rhs);
	const Rotation2D operator+(const Rotation2D &other);
    const Rotation2D operator-(const Rotation2D &other);

    double getCos() const;
    double getSin() const;
    double getTan() const;
    double getRadians() const;
    double getDegrees() const;
    void setRadians(const double &angle);
    void setDegrees(const double &angle);

    //////////////////////////////////////////////////////////////////////
    // @brief rotate rotation by another rotation
    //////////////////////////////////////////////////////////////////////
    Rotation2D rotateBy(const Rotation2D &other) const;

    //////////////////////////////////////////////////////////////////////
    // @brief inverse of rotation
    //////////////////////////////////////////////////////////////////////
    Rotation2D inverse() const;

private:
    //////////////////////////////////////////////////////////////////////
    // @brief normalize cos and sin to unit circle
    //////////////////////////////////////////////////////////////////////
    void normalize();

    double m_cos;
    double m_sin;
};
