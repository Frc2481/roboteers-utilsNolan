#ifndef TRAJECTORY_GENERATOR_1D_H
#define TRAJECTORY_GENERATOR_1D_H

#include <vector>
#include <string>

#define ONED_INTEGRATE_PATH_DIST_STEP 0.01 // (units)

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

class TrajectoryGenerator1D {
public:
    struct waypoint_t {
        double pos;
        double speed;
    };

    struct finalPathPoint_t {
        double time;    // timestamp (s)
        double pos;    // position (units)
        double dist;    // distance traveled along path (units)
        double vel;     // velocity (units/s)
        double accel;   // acceleration (units/s)
    };

    TrajectoryGenerator1D(
        std::vector<waypoint_t> &waypoints,
        double sampleRate,
        double maxSpeed,
        double maxAccel,
        double maxDeccel);

	~TrajectoryGenerator1D();

    void setWaypoints(std::vector<waypoint_t> &waypoints);
    void setSampleRate(unsigned sampleRate);
    void setMaxSpeed(double maxSpeed);
    void setMaxAccel(double maxAccel);
    void setMaxDeccel(double maxDeccel);
    void setWaypointsFilename(const std::string &waypointsFilename);
    void setPathFilename(const std::string &pathFilename);

    //////////////////////////////////////////////////////////////////////
    // @brief set to true if reference wraps around from rangeMax to
	//        rangeMin such as angle range of +/-180 deg
    //////////////////////////////////////////////////////////////////////
	void setIsContinous(
		bool isContinous,
		double rangeMin,
		double rangeMax);

    std::vector<finalPathPoint_t> getFinalPath() const;

    //////////////////////////////////////////////////////////////////////
    // @brief generate path from waypoints
    //////////////////////////////////////////////////////////////////////
    void generatePath();

    //////////////////////////////////////////////////////////////////////
    // @brief write output path to csv file
    //////////////////////////////////////////////////////////////////////
    void writePathToCSV() const;

    //////////////////////////////////////////////////////////////////////
    // @brief write temp path to csv file
    //////////////////////////////////////////////////////////////////////
    void writeTempPathToCSV() const;
    
    //////////////////////////////////////////////////////////////////////
    // @brief write combo path to csv file
    //////////////////////////////////////////////////////////////////////
    void writeComboPathToCSV() const;
    
    //////////////////////////////////////////////////////////////////////
    // @brief read waypoints from csv file
    //////////////////////////////////////////////////////////////////////
    void readWaypointsFromCSV();

private:
    std::vector<waypoint_t> m_waypoints;  // desired waypoints to travel through
    std::vector<finalPathPoint_t> m_tempPath; // trajectory generator temporary path
    std::vector<finalPathPoint_t> m_comboPath; // trajectory generator temporary path
    std::vector<finalPathPoint_t> m_finalPath; // generated path to follow
    unsigned m_sampleRate;  // sample rate of generated path points (Hz)
    double m_maxSpeed;  // max speed (units/s)
    double m_maxAccel;  // max acceleration (units/s)
    double m_maxDeccel; // max decceleration (units/s)
    double m_rangeMin;  // min range (units)
	double m_rangeMax;  // max range (units)
	bool m_isContinous; // flag indicating wraparound
    double m_totalPathDist; // total path distance (units)
    std::string m_waypointsFilename;    // waypoints input CSV filename
    std::string m_pathFilename; // path output CSV filename

    //////////////////////////////////////////////////////////////////////
    // @brief integrate path forward in time to calculate speed and acceleration
    // at given distance along path
    //////////////////////////////////////////////////////////////////////
    void integratePath(std::vector<finalPathPoint_t> &integratedPath, bool isBackward);
};

#endif // TRAJECTORY_GENERATOR_1D_H
