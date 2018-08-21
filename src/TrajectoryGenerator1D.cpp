#include "TrajectoryGenerator1D.h"
#include <math.h>

TrajectoryGenerator1D::TrajectoryGenerator1D()
    : m_waypoints(),
    m_tempPath(),
    m_comboPath(),
    m_finalPath(),
    m_sampleRate(50),
    m_maxSpeed(1),
    m_maxAccel(1),
    m_maxDeccel(-1),
    m_rangeMin(0),
	m_rangeMax(0),
	m_isContinous(false) {
}

TrajectoryGenerator1D::~TrajectoryGenerator1D() {
}

void TrajectoryGenerator1D::setWaypoints(std::vector<waypoint> &waypoints) {
    m_waypoints.clear();
    for(std::vector<waypoint>::iterator it = waypoints.begin(); it != waypoints.end(); ++it) {
        it->speed = abs(it->speed);
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

std::vector<TrajectoryGenerator1D::finalPathPoint> TrajectoryGenerator1D::getFinalPath() {
    return m_finalPath;
}

void TrajectoryGenerator1D::generatePath() {
    // @TODO: add path generation logic
    // @TODO: don't forget isContinous

    finalPathPoint tempPathGenPoint;

    // clear old path
    m_tempPath.clear();
    m_comboPath.clear();
    m_finalPath.clear();

    // calculate distance traveled along path
    // std::vector<double> tempPathDist;
    // std::vector<double> tempPathPos;
    m_totalPathDist = 0;
    tempPathGenPoint.pos = m_waypoints.front().pos;
    tempPathGenPoint.dist = m_totalPathDist;
    tempPathGenPoint.vel = sign(m_waypoints[1].pos - m_waypoints[0].pos) * m_waypoints.front().speed;
    m_tempPath.push_back(tempPathGenPoint);
    for(unsigned i = 1; i < m_waypoints.size(); ++i) {
        tempPathGenPoint.pos = m_waypoints[i].pos;
        tempPathGenPoint.dist = fabs(m_waypoints[i].pos - m_waypoints[i - 1].pos);
        tempPathGenPoint.vel = sign(m_waypoints[i].pos - m_waypoints[i - 1].pos) * m_waypoints[i].speed;
        m_tempPath.push_back(tempPathGenPoint);
    }
    
    // integrate path forward

    // integrate path backward

    // combine forward and backward paths with min speed

    // calculate path with respect to time

    // add first point to final path
    tempPathGenPoint.time = 0;
    tempPathGenPoint.dist = 0;
    tempPathGenPoint.vel = m_comboPath.front().vel;
    tempPathGenPoint.accel = 0;
    m_finalPath.push_back(tempPathGenPoint);

    // calculate final path

    // add final point to final path
}

void TrajectoryGenerator1D::integratePath(std::vector<finalPathPoint> &integratedPath {
    finalPathPoint tempPathGenPoint;

    // clear path
    integratedPath.clear();

    // add start point to path
    tempPathGenPoint.dist = 0;
    tempPathGenPoint.vel = m_tempPath.front().vel;
    integratedPath.push_back(tempPathGenPoint);

    // integrate path
    double accelSpeed;
    double pathSpeed;
    bool isNewTempPoint = false;
    
    unsigned i = 1;

    // @TODO: continue working here
    
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

        // hold and limit lateral slip speed through turn and limit individual wheel speed
        if((1 < i) && (i < (m_tempPath.size() - 2))) {
            if(!isBackward) {
                // check if in turn
                if((m_tempPath[i - 1].radCurve == m_tempPath[i].radCurve)
                    && (m_tempPath[i].radCurve != std::numeric_limits<double>::infinity())
                    && (m_tempPath[i - 1].radCurve != std::numeric_limits<double>::infinity())) {
                    latSlipSpeed = sqrt(m_maxCentripAccel * m_tempPath[i].radCurve);
                    limitWheelSpeed = m_maxSpeed / (1 + (1 / m_tempPath[i].radCurve) * (m_wheelTrack / 2));
                }
            }
            else {
                // check if in turn
                if((m_tempPath[i + 1].radCurve == m_tempPath[i].radCurve)
                    && (m_tempPath[i].radCurve != std::numeric_limits<double>::infinity())
                    && (m_tempPath[i + 1].radCurve != std::numeric_limits<double>::infinity())) {
                    latSlipSpeed = sqrt(m_maxCentripAccel * m_tempPath[i].radCurve);
                    limitWheelSpeed = m_maxSpeed / (1 + (1 / m_tempPath[i].radCurve) * (m_wheelTrack / 2));
                }
            }
        }
        else {
            latSlipSpeed = std::numeric_limits<double>::infinity();
            limitWheelSpeed = std::numeric_limits<double>::infinity();
        }

        // use minimum speed from all constraints
        double speed = std::min(std::min(std::min(std::min(accelSpeed, pathSpeed), latSlipSpeed), limitWheelSpeed), m_maxSpeed);

        // reverse path direction if needed
        if(m_isReverse) {
            speed = -speed;
        }

        // add speed to point and add to path
        tempPathGenPoint.vel = speed;
        integratedPath.push_back(tempPathGenPoint);
    }
    
    if(isBackward) {
        std::reverse(integratedPath.begin(), integratedPath.end());
    }
}