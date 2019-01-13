#ifndef TANK_DRIVE_PATH_GENERATOR_H
#define TANK_DRIVE_PATH_GENERATOR_H

#include <vector>
#include <string>

#define TANK_NUM_PHI_STEPS 301 // must be odd
#define TANK_INTEGRATE_PATH_DIST_STEP 1 // (in)

class TankDrivePathGenerator {
public:
    struct waypoint_t {
        double xPos;    // x position (in)
        double yPos;    // y position (in)
        double speed;   // speed (in/s)
        double maxDistThresh;   // max distance away that robot is allowed to travel from waypoint (in)
    };

    struct finalPathPoint_t {
        double time;    // timestamp (s)
        double xPos;    // x position (in)
        double yPos;    // y position (in)
        double yaw;     // yaw (deg)
        double dist;    // distance traveled along path (in)
        double vel;     // velocity (in/s)
        double accel;   // acceleration (in/s)
        double yawRate; // yaw rate (deg/s)
    };

    struct pathGenPoint_t {
        double xPos;    // x position (in)
        double yPos;    // y position (in)
        double vel;     // velocity (in/s)
        double radCurve;   // radius of curvature (in)
        double dist;    // distance traveled along path (in)
    };

    TankDrivePathGenerator(
        std::vector<waypoint_t> &waypoints,
        double sampleRate,
        double wheelTrack,
        double maxSpeed,
        double maxAccel,
        double maxDeccel,
        double maxCentripAccel);
        
    ~TankDrivePathGenerator();

    void setWaypoints(std::vector<waypoint_t> &waypoints);
    void setSampleRate(unsigned sampleRate);
    void setIsReverse(bool isReverse);
    void setWheelTrack(double wheelTrack);
    void setMaxSpeed(double maxSpeed);
    void setMaxAccel(double maxAccel);
    void setMaxDeccel(double maxDeccel);
    void setMaxCentripAccel(double maxCentripAccel);
    void setWaypointsFilename(const std::string &waypointsFilename);
    void setPathFilename(const std::string &pathFilename);

    std::vector<finalPathPoint_t> getFinalPath() const;

    //////////////////////////////////////////////////////////////////////
    // @brief generate path from waypoints for tank drive robot to follow
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
    std::vector<waypoint_t> m_waypoints;  // desired waypoints for robot to travel through
    std::vector<pathGenPoint_t> m_tempPath;  // path generator temporary path
    std::vector<finalPathPoint_t> m_comboPath; // path generator temporary path
    std::vector<finalPathPoint_t> m_finalPath; // generated path for robot to follow
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
    void integratePath(std::vector<pathGenPoint_t> &integratedPath, bool isBackward);
    
    //////////////////////////////////////////////////////////////////////
    // @brief acos() with safe bounds around +/-1 for floats
    //////////////////////////////////////////////////////////////////////
    double safeACos(double val) const;
};

#endif // TANK_DRIVE_PATH_GENERATOR_H
