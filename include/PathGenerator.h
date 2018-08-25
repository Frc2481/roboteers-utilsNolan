#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H

#include <vector>
#include <string>

#define MIN_ARC_RAD 0.1 // (in)
#define NUM_PHI_STEPS 51
#define INTEGRATE_PATH_DIST_STEP 0.01 // (in)

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

class PathGenerator {
public:
    PathGenerator();
    ~PathGenerator();

    struct waypoint {
        double xPos;    // x position (in)
        double yPos;    // y position (in)
        double speed;   // speed (in/s)
        double maxDistThresh;   // max distance away that robot is allowed to travel from waypoint (in)
    };

    struct finalPathPoint {
        double time;    // timestamp (s)
        double xPos;    // x position (in)
        double yPos;    // y position (in)
        double yaw;     // yaw (rad)
        double dist;    // distance traveled along path (in)
        double vel;     // velocity (in/s)
        double accel;   // acceleration (in/s)
        double yawRate; // yaw rate (rad/s)
    };

    struct pathGenPoint {
        double xPos;    // x position (in)
        double yPos;    // y position (in)
        double vel;     // velocity (in/s)
        double radCurve;   // radius of curvature (in)
        double dist;    // distance traveled along path (in)
    };

    void setWaypoints(std::vector<waypoint> &waypoints);
    void setSampleRate(const unsigned &sampleRate);
    void setIsReverse(const bool &isReverse);
    void setWheelTrack(const double &wheelTrack);
    void setMaxSpeed(const double &maxSpeed);
    void setMaxAccel(const double &maxAccel);
    void setMaxDeccel(const double &maxDeccel);
    void setMaxCentripAccel(const double &maxCentripAccel);
    void setWaypointsFilename(const std::string &waypointsFilename);
    void setPathFilename(const std::string &pathFilename);

    std::vector<finalPathPoint> getFinalPath();

    //////////////////////////////////////////////////////////////////////
    // @brief generate path from waypoints for tank drive robot to follow
    //////////////////////////////////////////////////////////////////////
    void generatePath();
    
    //////////////////////////////////////////////////////////////////////
    // @brief write output path to csv file
    //////////////////////////////////////////////////////////////////////
    void writePathToCSV();
    
    //////////////////////////////////////////////////////////////////////
    // @brief write temp path to csv file
    //////////////////////////////////////////////////////////////////////
    void writeTempPathToCSV();
    
    //////////////////////////////////////////////////////////////////////
    // @brief write combo path to csv file
    //////////////////////////////////////////////////////////////////////
    void writeComboPathToCSV();
    
    //////////////////////////////////////////////////////////////////////
    // @brief read waypoints from csv file
    //////////////////////////////////////////////////////////////////////
    void readWaypointsFromCSV();

private:
    std::vector<waypoint> m_waypoints;  // desired waypoints for robot to travel through
    std::vector<pathGenPoint> m_tempPath;  // path generator temporary path
    std::vector<finalPathPoint> m_comboPath; // path generator temporary path
    std::vector<finalPathPoint> m_finalPath; // generated path for robot to follow
    unsigned m_sampleRate;  // sample rate of generated path points (Hz)
    bool m_isReverse;   // reverse path direction
    double m_wheelTrack;    // width between left and right wheels (in)
    double m_maxSpeed;  // max speed of robot (in/s)
    double m_maxAccel;  // max acceleration of robot (in/s^2)
    double m_maxDeccel; // max decceleration of robot (in/s^2)
    double m_maxCentripAccel;   // max centripetal acceleration of robot (in/s^2)
    double m_totalPathDist; // total path distance (in)
    std::string m_waypointsFilename;    // waypoints input CSV filename
    std::string m_pathFilename; // path output CSV filename
    
    //////////////////////////////////////////////////////////////////////
    // @brief integrate path forward in time to calculate speed and acceleration
    // at given distance along path
    //////////////////////////////////////////////////////////////////////
    void integratePath(std::vector<pathGenPoint> &integratedPath, bool isBackward);
    
    //////////////////////////////////////////////////////////////////////
    // @brief acos() with safe bounds around +/-1 for floats
    //////////////////////////////////////////////////////////////////////
    double safeACos(double val);
};

#endif // PATH_GENERATOR_H