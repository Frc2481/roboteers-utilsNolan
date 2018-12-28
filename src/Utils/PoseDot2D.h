//////////////////////////////////////////////////////////////////////
// 2D rigid body velocity
//////////////////////////////////////////////////////////////////////
#ifndef POSE_DOT_2D_H
#define POSE_DOT_2D_H

class PoseDot2D {
public:
	PoseDot2D();
	PoseDot2D(double xVel, double yVel, double yawRateDegPerSec);
	PoseDot2D(const PoseDot2D &other);
    ~PoseDot2D();

    PoseDot2D& operator=(const PoseDot2D &other);

    double getXVel();
    double getYVel();
	double getYawRateDegPerSec();

private:
    double m_xVel;
    double m_yVel;
    double m_yawRateDegPerSec;
};

#endif // POSE_DOT_2D_H
