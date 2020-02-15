#ifndef POSE_DOT_2D_H
#define POSE_DOT_2D_H

class PoseDot2D {
public:
	PoseDot2D();
	PoseDot2D(double xVel, double yVel, double yawRate);
	PoseDot2D(const PoseDot2D &other);
    ~PoseDot2D();

    PoseDot2D& operator=(const PoseDot2D &other);

    double getXVel();
    double getYVel();
	double getYawRate();

private:
    double m_xVel;      // in/s
    double m_yVel;      // in/s
    double m_yawRate;   // deg/s
};

#endif // POSE_DOT_2D_H
