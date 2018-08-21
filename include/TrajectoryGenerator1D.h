//////////////////////////////////////////////////////////////////////
// 1D trajectory generator
//////////////////////////////////////////////////////////////////////
#pragma once
#include <vector>

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

class TrajectoryGenerator1D {
public:
    TrajectoryGenerator1D();
	~TrajectoryGenerator1D();

    struct waypoint {
        double pos;
        double speed;
    };

    struct finalPathPoint {
        double time;    // timestamp (s)
        double pos;    // position (units)
        double dist;    // distance traveled along path (units)
        double vel;     // velocity (units/s)
        double accel;   // acceleration (units/s)
    };

    void setWaypoints(std::vector<waypoint> &waypoints);
    void setSampleRate(const unsigned &sampleRate);
    void setMaxSpeed(const double &maxSpeed);
    void setMaxAccel(const double &maxAccel);
    void setMaxDeccel(const double &maxDeccel);

    //////////////////////////////////////////////////////////////////////
    // @brief set to true if reference wraps around from rangeMax to
	//        rangeMin such as angle range of +/-180 deg
    //////////////////////////////////////////////////////////////////////
	void setIsContinous(
		const bool &isContinous,
		const double &rangeMin,
		const double &rangeMax);

    std::vector<finalPathPoint> getFinalPath();

    //////////////////////////////////////////////////////////////////////
    // @brief generate path from waypoints
    //////////////////////////////////////////////////////////////////////
    void generatePath();

private:
    std::vector<waypoint> m_waypoints;  // desired waypoints to travel through
    std::vector<finalPathPoint> m_tempPath; // trajectory generator temporary path
    std::vector<finalPathPoint> m_comboPath; // trajectory generator temporary path
    std::vector<finalPathPoint> m_finalPath; // generated path to follow
    unsigned m_sampleRate;  // sample rate of generated path points (Hz)
    double m_maxSpeed;  // max speed (units/s)
    double m_maxAccel;  // max acceleration (units/s)
    double m_maxDeccel; // max decceleration (units/s)
    double m_rangeMin;  // min range (units)
	double m_rangeMax;  // max range (units)
	bool m_isContinous; // flag indicating wraparound
    double m_totalPathDist; // total path distance (units)

    //////////////////////////////////////////////////////////////////////
    // @brief integrate path forward in time to calculate speed and acceleration
    // at given distance along path
    //////////////////////////////////////////////////////////////////////
    void TrajectoryGenerator1D::integratePath(std::vector<finalPathPoint> &integratedPath
}