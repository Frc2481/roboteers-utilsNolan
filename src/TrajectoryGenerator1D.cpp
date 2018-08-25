#include "TrajectoryGenerator1D.h"
#include <math.h>
#include <limits>
#include <algorithm>
#include "Interpolate.h"
#include <iostream>
#include <fstream>
#include <stdio.h>

TrajectoryGenerator1D::TrajectoryGenerator1D(
    std::vector<waypoint_t> &waypoints,
    const double &sampleRate,
    const double &maxSpeed,
    const double &maxAccel,
    const double &maxDeccel)

    : m_tempPath(),
    m_comboPath(),
    m_finalPath(),
    m_sampleRate(sampleRate),
    m_maxSpeed(maxSpeed),
    m_maxAccel(maxAccel),
    m_maxDeccel(maxDeccel),
    m_rangeMin(0),
	m_rangeMax(0),
	m_isContinous(false),
    m_waypointsFilename("tempWaypoints.csv"),
    m_pathFilename("tempFinalPath.csv") {

    setWaypoints(waypoints);
}

TrajectoryGenerator1D::~TrajectoryGenerator1D() {
}

void TrajectoryGenerator1D::setWaypoints(std::vector<waypoint_t> &waypoints) {
    m_waypoints.clear();
    for(std::vector<waypoint_t>::iterator it = waypoints.begin(); it != waypoints.end(); ++it) {
        it->speed = abs(it->speed);
        // check wraparound
        if(m_isContinous && ((it->pos > m_rangeMax) || (it->pos < m_rangeMin))) {
            it->pos = m_rangeMin - 
                (m_rangeMax - m_rangeMin) * cos(it->pos / (m_rangeMax - m_rangeMin) * 2 * M_PI);
        }

        m_waypoints.push_back(*it);
    }
}

void TrajectoryGenerator1D::setSampleRate(const unsigned &sampleRate) {
    m_sampleRate = sampleRate;
}

void TrajectoryGenerator1D::setMaxSpeed(const double &maxSpeed) {
    if(maxSpeed != 0) {
        m_maxSpeed = abs(maxSpeed);
    }
}

void TrajectoryGenerator1D::setMaxAccel(const double &maxAccel) {
    if(maxAccel != 0) {
        m_maxAccel = abs(maxAccel);
    }
}

void TrajectoryGenerator1D::setMaxDeccel(const double &maxDeccel) {
    if(maxDeccel != 0) {
        m_maxDeccel = -abs(maxDeccel);
    }
}

void TrajectoryGenerator1D::setIsContinous(
    const bool &isContinous,
    const double &rangeMin,
    const double &rangeMax) {
    
    if(!isContinous) {
        m_rangeMin = 0;
        m_rangeMax = 0;
        return;
    }

    if(rangeMin > rangeMax) {
        return;
    }
    
    m_isContinous = isContinous;
    m_rangeMin = rangeMin;
    m_rangeMax = rangeMax;
}

std::vector<TrajectoryGenerator1D::finalPathPoint_t> TrajectoryGenerator1D::getFinalPath() {
    return m_finalPath;
}

void TrajectoryGenerator1D::generatePath() {
    finalPathPoint_t tempPathGenPoint;

    // clear old path
    m_tempPath.clear();
    m_comboPath.clear();
    m_finalPath.clear();

    // calculate direction of travel
    double difference = m_waypoints[1].pos - m_waypoints[0].pos;
    if(m_isContinous && (fabs(difference) > (m_rangeMax - m_rangeMin) / 2)) {
        if(difference > 0) {
            difference = difference - (m_rangeMax - m_rangeMin);
        }
        else {
            difference = difference + (m_rangeMax - m_rangeMin);
        }
    }
    int directionSign = sign(difference);

    // calculate distance traveled along path
    std::vector<double> tempPathDist;
    std::vector<double> tempPathPos;
    m_totalPathDist = 0;
    tempPathGenPoint.pos = m_waypoints.front().pos;
    tempPathGenPoint.dist = m_totalPathDist;
    tempPathGenPoint.vel = directionSign * m_waypoints.front().speed;
    m_tempPath.push_back(tempPathGenPoint);
    tempPathDist.push_back(m_tempPath.front().dist);
    tempPathPos.push_back(m_tempPath.front().pos);
    for(unsigned i = 1; i < m_waypoints.size(); ++i) {
        // unwrap position to make continous
        double difference = m_waypoints[i].pos - m_waypoints[i - 1].pos;
        if(m_isContinous && (fabs(difference) > (m_rangeMax - m_rangeMin) / 2)) {
            if(difference > 0) {
                difference = difference - (m_rangeMax - m_rangeMin);
            }
            else {
                difference = difference + (m_rangeMax - m_rangeMin);
            }
        }
        tempPathGenPoint.pos = m_tempPath[i - 1].pos + difference;
        m_totalPathDist += fabs(difference);
        tempPathGenPoint.dist = m_totalPathDist;
        tempPathGenPoint.vel = directionSign * m_waypoints[i].speed;
        m_tempPath.push_back(tempPathGenPoint);

        // store temp path dist, pos in separate vectors
        tempPathDist.push_back(m_tempPath[i].dist);
        tempPathPos.push_back(m_tempPath[i].pos);
    }
    
    // integrate path forward
    std::vector<finalPathPoint_t> fwdPath;
    integratePath(fwdPath, false);

    // integrate path backward
    std::vector<finalPathPoint_t> bwdPath;
    integratePath(bwdPath, true);

    // combine forward and backward paths with min speed
    for(unsigned i = 0; i < fwdPath.size(); ++i) {
        tempPathGenPoint.dist = fwdPath[i].dist;
        tempPathGenPoint.vel = std::min(fwdPath[i].vel, bwdPath[i].vel);
        m_comboPath.push_back(tempPathGenPoint);
    }

    // calculate path with respect to time
    std::vector<double> comboPathTime;
    std::vector<double> comboPathDist;
    std::vector<double> comboPathVel;
    m_comboPath.front().time = 0;
    comboPathTime.push_back(m_comboPath.front().time);
    comboPathDist.push_back(m_comboPath.front().dist);
    comboPathVel.push_back(m_comboPath.front().vel);
    for(unsigned i = 1; i < m_comboPath.size(); ++i) {
        // check divide by zero
        if((m_comboPath[i].vel + m_comboPath[i - 1].vel) != 0) {
            m_comboPath[i].time = m_comboPath[i - 1].time
                + 2 * (m_comboPath[i].dist - m_comboPath[i - 1].dist) / abs(m_comboPath[i].vel + m_comboPath[i - 1].vel);
        }
        else {
            m_comboPath[i].time = m_comboPath[i - 1].time;
        }

        // store combo path time, dist, vel in separate vectors
        comboPathTime.push_back(m_comboPath[i].time);
        comboPathDist.push_back(m_comboPath[i].dist);
        comboPathVel.push_back(m_comboPath[i].vel);
    }
    
    // add first point to final path
    tempPathGenPoint.time = 0;
    tempPathGenPoint.dist = 0;
    tempPathGenPoint.vel = m_comboPath.front().vel;
    tempPathGenPoint.accel = 0;
    tempPathGenPoint.pos = m_tempPath.front().pos;
    m_finalPath.push_back(tempPathGenPoint);

    // calculate final path
    while(tempPathGenPoint.time <= (m_comboPath.back().time - (1 / (double)m_sampleRate))) {
        tempPathGenPoint.time += 1 / (double)m_sampleRate;
        tempPathGenPoint.dist = interpolate::interp(comboPathTime, comboPathDist, tempPathGenPoint.time, false);
        tempPathGenPoint.vel = interpolate::interp(comboPathTime, comboPathVel, tempPathGenPoint.time, false);
        tempPathGenPoint.accel = (tempPathGenPoint.vel - m_finalPath.back().vel) * m_sampleRate;
        
        tempPathGenPoint.pos = interpolate::interp(tempPathDist, tempPathPos, tempPathGenPoint.dist, false);
        // check wraparound
        if(m_isContinous && ((tempPathGenPoint.pos > m_rangeMax) || (tempPathGenPoint.pos < m_rangeMin))) {
            tempPathGenPoint.pos = m_rangeMin - 
                (m_rangeMax - m_rangeMin) * cos(tempPathGenPoint.pos / (m_rangeMax - m_rangeMin) * 2 * M_PI);
        }

        m_finalPath.push_back(tempPathGenPoint);
    }

    // add final point to final path
    tempPathGenPoint.time = m_comboPath.back().time;
    tempPathGenPoint.dist = m_comboPath.back().dist;
    tempPathGenPoint.vel = m_comboPath.back().vel;
    tempPathGenPoint.accel = 0;
    tempPathGenPoint.pos = m_tempPath.back().pos;
    // check wraparound
    if(m_isContinous && ((tempPathGenPoint.pos > m_rangeMax) || (tempPathGenPoint.pos < m_rangeMin))) {
        tempPathGenPoint.pos = m_rangeMin - 
            (m_rangeMax - m_rangeMin) * cos(tempPathGenPoint.pos / (m_rangeMax - m_rangeMin) * 2 * M_PI);
    }
    m_finalPath.push_back(tempPathGenPoint);
}

void TrajectoryGenerator1D::writePathToCSV() {
    std::remove(m_pathFilename.c_str());
    
    std::ofstream myFile;
    myFile.open(m_pathFilename);
    myFile << "time (s), pos (units), dist (units), vel (units/s), accel (units/s^2)\n";
    
    for(unsigned i = 0; i < m_finalPath.size(); ++i) {
        myFile << m_finalPath[i].time << ",";
        myFile << m_finalPath[i].pos << ",";
        myFile << m_finalPath[i].dist << ",";
        myFile << m_finalPath[i].vel << ",";
        myFile << m_finalPath[i].accel;
        myFile << "\n";
    }

    myFile.close();
}

void TrajectoryGenerator1D::writeTempPathToCSV() {
    std::remove("tempPath.csv");
    
    std::ofstream myFile;
    myFile.open("tempPath.csv");
    myFile << "time (s), pos (units), dist (units), vel (units/s), accel (units/s^2)\n";
    
    for(unsigned i = 0; i < m_tempPath.size(); ++i) {
        myFile << m_tempPath[i].time << ",";
        myFile << m_tempPath[i].pos << ",";
        myFile << m_tempPath[i].dist << ",";
        myFile << m_tempPath[i].vel << ",";
        myFile << m_tempPath[i].accel;
        myFile << "\n";
    }

    myFile.close();
}

void TrajectoryGenerator1D::writeComboPathToCSV() {
    std::remove("tempComboPath.csv");
    
    std::ofstream myFile;
    myFile.open("tempComboPath.csv");
    myFile << "time (s), pos (units), dist (units), vel (units/s), accel (units/s^2)\n";
    
    for(unsigned i = 0; i < m_comboPath.size(); ++i) {
        myFile << m_comboPath[i].time << ",";
        myFile << m_comboPath[i].pos << ",";
        myFile << m_comboPath[i].dist << ",";
        myFile << m_comboPath[i].vel << ",";
        myFile << m_comboPath[i].accel;
        myFile << "\n";
    }

    myFile.close();
}

void TrajectoryGenerator1D::readWaypointsFromCSV() {
    std::vector<waypoint_t> waypoints;
    waypoint_t tempWaypoint;
    std::string header;
    char delim;
    
    // open file
    std::ifstream myFile;
    myFile.open(m_waypointsFilename, std::ifstream::in);

    // read config
    unsigned sampleRate;
    getline(myFile, header, ',');
    myFile >> sampleRate;
    setSampleRate(sampleRate);
    
    double maxSpeed;
    getline(myFile, header, ',');
    myFile >> maxSpeed;
    setMaxSpeed(maxSpeed);
    
    double maxAccel;
    getline(myFile, header, ',');
    myFile >> maxAccel;
    setMaxAccel(maxAccel);
    
    double maxDeccel;
    getline(myFile, header, ',');
    myFile >> maxDeccel;
    setMaxDeccel(maxDeccel);

    bool isContinous;
    getline(myFile, header, ',');
    myFile >> isContinous;

    double minRange;
    getline(myFile, header, ',');
    myFile >> minRange;

    double maxRange;
    getline(myFile, header, ',');
    myFile >> maxRange;
    setIsContinous(isContinous, minRange, maxRange);
    
    // read data
    getline(myFile, header); // skip \n
    getline(myFile, header); // skip blank line
    getline(myFile, header); // skip headers
    
    while(myFile >> tempWaypoint.pos >> delim
                >> tempWaypoint.speed) {
        waypoints.push_back(tempWaypoint);
    }
    
    // close file
    myFile.close();
    
    // set waypoints
    setWaypoints(waypoints);
}

void TrajectoryGenerator1D::integratePath(std::vector<finalPathPoint_t> &integratedPath, bool isBackward) {
    finalPathPoint_t tempPathGenPoint;

    // clear path
    integratedPath.clear();

    // add start point to path
    if(!isBackward) {
        tempPathGenPoint.dist = 0;
        tempPathGenPoint.vel = m_tempPath.front().vel;
    }
    else {
        tempPathGenPoint.dist = m_totalPathDist;
        tempPathGenPoint.vel = m_tempPath.back().vel;
    }
    integratedPath.push_back(tempPathGenPoint);

    // integrate path
    double accelSpeed;
    double pathSpeed;
    bool isNewTempPoint = false;
    unsigned i;
    
    if(!isBackward) {
        i = 1;
    }
    else {
        i = m_tempPath.size() - 2;
    }
    
    while(((integratedPath.back().dist < m_totalPathDist) && !isBackward)
          || ((integratedPath.back().dist > 0) && isBackward)) {
        // assume that sample rate is high enough so that temp path points do not need skipped
        if(!isBackward) {
            if(tempPathGenPoint.dist > m_tempPath[i].dist) {
                i++;
                isNewTempPoint = true;
            }
        }
        else {
            if(tempPathGenPoint.dist < m_tempPath[i].dist) {
                i--;
                isNewTempPoint = true;
            }
        }
        
        // increment distance traveled and add to point
        if(!isBackward) {
            tempPathGenPoint.dist += INTEGRATE_PATH_DIST_STEP;
        }
        else {
            tempPathGenPoint.dist -= INTEGRATE_PATH_DIST_STEP;
        }

        // calculate acceleration speed
        if(!isBackward) {
            accelSpeed = sqrt(pow(tempPathGenPoint.vel, 2) + 2 * m_maxAccel * INTEGRATE_PATH_DIST_STEP);
        }
        else {
            accelSpeed = sqrt(pow(tempPathGenPoint.vel, 2) - 2 * m_maxDeccel * INTEGRATE_PATH_DIST_STEP);
        }

        // get path speed
        if(isNewTempPoint) {
            if(!isBackward) {
                pathSpeed = m_tempPath[i - 1].vel;
            }
            else {
                pathSpeed = m_tempPath[i + 1].vel;
            }
            isNewTempPoint = false;
        }
        else {
            pathSpeed = std::numeric_limits<double>::infinity();
        }

        // use minimum speed from all constraints
        double speed = std::min(accelSpeed, pathSpeed);

        // add speed to point and add to path
        tempPathGenPoint.vel = speed;
        integratedPath.push_back(tempPathGenPoint);
    }
    
    if(isBackward) {
        std::reverse(integratedPath.begin(), integratedPath.end());
    }
}