#ifndef ROTATION_2D_H
#define ROTATION_2D_H

class Rotation2D {
public:
    Rotation2D();
    Rotation2D(double cos, double sin);
    Rotation2D(const Rotation2D &other);
    ~Rotation2D();
    static Rotation2D fromRadians(double angle);
    static Rotation2D fromDegrees(double angle);

    Rotation2D& operator=(const Rotation2D &rhs);
    Rotation2D& operator+=(const Rotation2D &rhs);
    Rotation2D& operator-=(const Rotation2D &rhs);
	Rotation2D operator+(const Rotation2D &other) const;
    Rotation2D operator-(const Rotation2D &other) const;

    double getCos() const;
    double getSin() const;
    double getTan() const;
    double getRadians() const;
    double getDegrees() const;
    
    void setRadians(double angle);
    void setDegrees(double angle);

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

#endif // ROTATION_2D_H 
