//////////////////////////////////////////////////////////////////////
// library for 2D rigid body rotations
//////////////////////////////////////////////////////////////////////
#pragma once

class Rotation2D {
public:
    //////////////////////////////////////////////////////////////////////
    // constructor
    //////////////////////////////////////////////////////////////////////
    Rotation2D();

    //////////////////////////////////////////////////////////////////////
    // constructor
    // cos          cosine of angle
    // sin          sine of angle
    //////////////////////////////////////////////////////////////////////
    Rotation2D(const double &cos, const double &sin);

    //////////////////////////////////////////////////////////////////////
    // constructor
    //////////////////////////////////////////////////////////////////////
    Rotation2D(const Rotation2D &other);

    //////////////////////////////////////////////////////////////////////
    // destructor
    //////////////////////////////////////////////////////////////////////
    ~Rotation2D();

    //////////////////////////////////////////////////////////////////////
    // create rotation from angle in rad
    // angle        angle in rad
    // return       rotation
    //////////////////////////////////////////////////////////////////////
    static Rotation2D fromRadians(const double &angle);

    //////////////////////////////////////////////////////////////////////
    // create rotation from angle in deg
    // angle        angle in deg
    // return       rotation
    //////////////////////////////////////////////////////////////////////
    static Rotation2D fromDegrees(const double &angle);

    //////////////////////////////////////////////////////////////////////
    // overloaded assignment operator
    //////////////////////////////////////////////////////////////////////
    Rotation2D& operator=(const Rotation2D &rhs);

    //////////////////////////////////////////////////////////////////////
    // overloaded binary addition operator
    //////////////////////////////////////////////////////////////////////
    Rotation2D& operator+=(const Rotation2D &rhs);

    //////////////////////////////////////////////////////////////////////
    // overloaded binary subtraction operator
    //////////////////////////////////////////////////////////////////////
    Rotation2D& operator-=(const Rotation2D &rhs);

    //////////////////////////////////////////////////////////////////////
    // get cosine of angle encoded by rotation
    // return       cosine of angle
    //////////////////////////////////////////////////////////////////////
    double getCos() const;

    //////////////////////////////////////////////////////////////////////
    // get sine of angle encoded by rotation
    // return       sine of angle
    //////////////////////////////////////////////////////////////////////
    double getSin() const;

    //////////////////////////////////////////////////////////////////////
    // get tangent of angle encoded by rotation
    // return       tangent of angle
    //////////////////////////////////////////////////////////////////////
    double getTan() const;

    //////////////////////////////////////////////////////////////////////
    // get angle encoded by rotation in rad
    // return       angle in rad
    //////////////////////////////////////////////////////////////////////
    double getRadians() const;

    //////////////////////////////////////////////////////////////////////
    // get angle encoded by rotation in deg
    // return       angle in deg
    //////////////////////////////////////////////////////////////////////
    double getDegrees() const;

    //////////////////////////////////////////////////////////////////////
    // set angle encoded by rotation in rad
    // angle       angle in rad
    //////////////////////////////////////////////////////////////////////
    void setRadians(const double &angle);

    //////////////////////////////////////////////////////////////////////
    // set angle encoded by rotation in deg
    // angle       angle in deg
    //////////////////////////////////////////////////////////////////////
    void setDegrees(const double &angle);


    //////////////////////////////////////////////////////////////////////
    // rotate rotation by another rotation
    // other        rotation to rotate by
    // return       rotated rotation
    //////////////////////////////////////////////////////////////////////
    Rotation2D rotateBy(const Rotation2D &other) const;

    //////////////////////////////////////////////////////////////////////
    // invert rotation
    // return       inverted rotation
    //////////////////////////////////////////////////////////////////////
    Rotation2D inverse() const;


    //////////////////////////////////////////////////////////////////////
    // overloaded binary addition operator
    //////////////////////////////////////////////////////////////////////
    const Rotation2D operator+(const Rotation2D &other);

    //////////////////////////////////////////////////////////////////////
    // overloaded binary subtraction operator
    //////////////////////////////////////////////////////////////////////
    const Rotation2D operator-(const Rotation2D &other);

private:
    //////////////////////////////////////////////////////////////////////
    // normalize cos and sin to unit circle
    //////////////////////////////////////////////////////////////////////
    void normalize();

    double m_cos;
    double m_sin;
};
