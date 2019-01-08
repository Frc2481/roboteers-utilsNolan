#include "SwerveDrivePathGenerator.h"
#include <math.h>
#include <limits>
#include <algorithm>
#include "Interpolate.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include "normalizeToRange.h"
#include "Translation2D.h"
#include "Sign.h"

SwerveDrivePathGenerator::SwerveDrivePathGenerator(
    std::vector<waypoint_t> &waypoints,
    double sampleRate,
    double wheelTrack,
	double wheelBase,
    double maxSpeed,
    double maxAccel,
    double maxDeccel,
    double maxCentripAccel)

    : m_tempPath(),
    m_comboPath(),
    m_finalPath(),
    m_sampleRate(sampleRate),
    m_isReverse(false),
    m_wheelTrack(wheelTrack),
	m_wheelBase(wheelBase),
    m_maxSpeed(maxSpeed),
    m_maxAccel(maxAccel),
    m_maxDeccel(maxDeccel),
    m_maxCentripAccel(maxCentripAccel),
    m_waypointsFilename("PathGeneratorWaypoints.csv"),
    m_pathFilename("PathGeneratorFinalPath.csv") {
    
    setWaypoints(waypoints);
}

SwerveDrivePathGenerator::~SwerveDrivePathGenerator() {
}

void SwerveDrivePathGenerator::setWaypoints(std::vector<waypoint_t> &waypoints) {
    m_waypoints.clear();
    for(std::vector<waypoint_t>::iterator it = waypoints.begin(); it != waypoints.end(); ++it) {
        it->speed = abs(it->speed);
        it->maxDistThresh = abs(it->maxDistThresh);
        m_waypoints.push_back(*it);
    }
}

void SwerveDrivePathGenerator::setSampleRate(unsigned sampleRate) {
    m_sampleRate = sampleRate;
}

void SwerveDrivePathGenerator::setIsReverse(bool isReverse) {
    m_isReverse = isReverse;
}

void SwerveDrivePathGenerator::setWheelTrack(double wheelTrack) {
    if(wheelTrack != 0) {
        m_wheelTrack = abs(wheelTrack);
    }
}

void SwerveDrivePathGenerator::setWheelBase(double wheelBase) {
    if(wheelBase != 0) {
        m_wheelBase = abs(wheelBase);
    }
}

void SwerveDrivePathGenerator::setMaxSpeed(double maxSpeed) {
    if(maxSpeed != 0) {
        m_maxSpeed = abs(maxSpeed);
    }
}

void SwerveDrivePathGenerator::setMaxAccel(double maxAccel) {
    if(maxAccel != 0) {
        m_maxAccel = abs(maxAccel);
    }
}

void SwerveDrivePathGenerator::setMaxDeccel(double maxDeccel) {
    if(maxDeccel != 0) {
        m_maxDeccel = -abs(maxDeccel);
    }
}

void SwerveDrivePathGenerator::setMaxCentripAccel(double maxCentripAccel) {
    if(maxCentripAccel != 0) {
        m_maxCentripAccel = abs(maxCentripAccel);
    }
}

void SwerveDrivePathGenerator::setWaypointsFilename(const std::string &waypointsFilename) {
    m_waypointsFilename = waypointsFilename;
}

void SwerveDrivePathGenerator::setPathFilename(const std::string &pathFilename) {
    m_pathFilename = pathFilename;
}

std::vector<SwerveDrivePathGenerator::finalPathPoint_t> SwerveDrivePathGenerator::getFinalPath() const {
    return m_finalPath;
}

void SwerveDrivePathGenerator::generatePath() {
    pathGenPoint_t tempPathGenPoint;
	comboPathPoint_t tempComboPathPoint;
    finalPathPoint_t tempFinalPathPoint;

    // clear old path
    m_tempPath.clear();
    m_comboPath.clear();
    m_finalPath.clear();

    // add start point to path
    tempPathGenPoint.xPos = m_waypoints.front().xPos;
    tempPathGenPoint.yPos = m_waypoints.front().yPos;
    tempPathGenPoint.radCurve = std::numeric_limits<double>::infinity();
    tempPathGenPoint.vel = m_waypoints.front().speed;
    tempPathGenPoint.dist = 0;
    tempPathGenPoint.yaw = m_waypoints.front().yaw;
    m_tempPath.push_back(tempPathGenPoint);
    
    // generate path trajectory    
    for(unsigned i = 0; i < (m_waypoints.size() - 2); ++i) {
        // get waypoint max distance threshold and speed
        double maxDistThresh = m_waypoints[i + 1].maxDistThresh;
        double speed = m_waypoints[i + 1].speed;
        double yaw = m_waypoints[i + 1].yaw;

        // get 3 consecutive points
        Translation2D p1(m_waypoints[i].xPos, m_waypoints[i].yPos); // current waypoint
        Translation2D p2(m_waypoints[i + 1].xPos, m_waypoints[i + 1].yPos); // next waypoint
        Translation2D p3(m_waypoints[i + 2].xPos, m_waypoints[i + 2].yPos); // next next waypoint

        // get vectors between points
        Translation2D v21 = p1 - p2;
        Translation2D v23 = p3 - p2;

        // check if redundant points
        double theta;
        if((v21.norm() == 0) || (v23.norm() == 0)) {
            continue;    // else skip waypoint
        }
        else {
            theta = safeACos(v21.dot(v23) / (v21.norm() * v23.norm())); // angle between v21 and v23
        }

        // check if points lie on same line
        if((theta == M_PI) || (theta == 0)) {
            // do not insert rounded corner
            tempPathGenPoint.xPos = p2.getX();
            tempPathGenPoint.yPos = p2.getY();
            tempPathGenPoint.radCurve = std::numeric_limits<double>::infinity();
            tempPathGenPoint.vel = speed;
            tempPathGenPoint.yaw = yaw;
            m_tempPath.push_back(tempPathGenPoint);
        }
        else {
            // calculate arc between points
            double arcRad = maxDistThresh * sin(theta / 2.0) / (1 - sin(theta / 2.0));
            if(arcRad < (m_wheelTrack / 2.0)) { // remove velocity spike at discontinuity
                arcRad = (m_wheelTrack / 2.0);
            }
            double arcHeight = arcRad * (1 - cos((M_PI - theta) / 2.0));
            double arcChordLen = 2 * arcRad * sin((M_PI - theta) / 2.0);
            double straightDist = sqrt(pow((maxDistThresh + arcHeight), 2) + pow((arcChordLen / 2.0), 2));
                // distance between p2 and point where path transitions from straight to rounded corner

            // limit arc radius if too large for distance between points
            double limitDist = std::min(v21.norm(), v23.norm()) / 2.0;
            if(straightDist > limitDist) {
                straightDist = limitDist;
                arcChordLen = 2 * straightDist * sin(theta / 2.0);
                arcRad = arcChordLen * sin(theta / 2.0) / sin(M_PI - theta);
                arcHeight = arcRad * (1 - cos((M_PI - theta) / 2.0));
                maxDistThresh = straightDist * cos(theta / 2.0) - arcHeight;
            }

            // calculate points on arc that are tangent to v21 and v23
            Translation2D v24 = v21.scaleBy(1 / (v21.norm() * straightDist));
            Translation2D v25 = v23.scaleBy(1 / (v23.norm() * straightDist));
            Translation2D p4 = p2 + v24;
            Translation2D p5 = p2 + v25;
            
            // calculate center point of arc
            Translation2D v26 = (v21.scaleBy(1 / v21.norm()) + v23.scaleBy(1 / v23.norm())).scaleBy(0.5);
            Translation2D p6 = p2 + v26.scaleBy((maxDistThresh + arcRad) / v26.norm()); // center point of arc

            // generate points along arc
            double phiStep = (M_PI - theta) / SWERVE_NUM_PHI_STEPS;
            double phi = Sign::sign(v21.cross(v23)) * (M_PI - theta) / 2.0;
            for(unsigned j = 1; j <= SWERVE_NUM_PHI_STEPS; j++) {
                Translation2D p7 = p6 - v26.rotateBy(Rotation2D::fromRadians(phi)).scaleBy(arcRad / v26.norm());
                tempPathGenPoint.xPos = p7.getX();
                tempPathGenPoint.yPos = p7.getY();

                // check if arc endpoint
                if((j == 1) || (j == SWERVE_NUM_PHI_STEPS)) {
                    tempPathGenPoint.radCurve = std::numeric_limits<double>::infinity();
                }
                else {
                    tempPathGenPoint.radCurve = arcRad;
                }

                // check if waypoint
                if(j == ceil(SWERVE_NUM_PHI_STEPS / 2.0)) {
                    tempPathGenPoint.vel = speed;
                    tempPathGenPoint.yaw = yaw;
                }
                else {
                    tempPathGenPoint.vel = std::numeric_limits<double>::infinity();
                    tempPathGenPoint.yaw = std::numeric_limits<double>::infinity();
                }

                // add point to path gen points and increment phi
                m_tempPath.push_back(tempPathGenPoint);
                phi = phi - Sign::sign(v21.cross(v23)) * phiStep;
            }
        }
    }

    // add end point to path
    tempPathGenPoint.xPos = m_waypoints.back().xPos;
    tempPathGenPoint.yPos = m_waypoints.back().yPos;
    tempPathGenPoint.radCurve = std::numeric_limits<double>::infinity();
    tempPathGenPoint.vel = m_waypoints.back().speed;
    tempPathGenPoint.yaw = m_waypoints.back().yaw;
    m_tempPath.push_back(tempPathGenPoint);

    // calculate distance traveled along path
    std::vector<double> tempPathDist;
    std::vector<double> tempPathXPos;
    std::vector<double> tempPathYPos;
    std::vector<double> tempPathYaw;
    m_totalPathDist = 0;
    m_tempPath.front().dist = m_totalPathDist;
    tempPathDist.push_back(m_tempPath.front().dist);
    tempPathXPos.push_back(m_tempPath.front().xPos);
    tempPathYPos.push_back(m_tempPath.front().yPos);
    tempPathYaw.push_back(m_tempPath.front().yaw);
    for(unsigned i = 1; i < m_tempPath.size(); ++i) {
        Translation2D v21 = Translation2D(m_tempPath[i].xPos, m_tempPath[i].yPos)
            - Translation2D(m_tempPath[i - 1].xPos, m_tempPath[i - 1].yPos);
        m_totalPathDist += v21.norm();
        m_tempPath[i].dist = m_totalPathDist;

        // store temp path dist, xPos, yPos, yaw in separate vectors
        tempPathDist.push_back(m_tempPath[i].dist);
        tempPathXPos.push_back(m_tempPath[i].xPos);
        tempPathYPos.push_back(m_tempPath[i].yPos);
        tempPathYaw.push_back(m_tempPath[i].yaw);
    }

    // integrate path forward
    std::vector<pathGenPoint_t> fwdPath;
    integratePath(fwdPath, false);

    // integrate path backward
    std::vector<pathGenPoint_t> bwdPath;
    integratePath(bwdPath, true);
    
    // combine forward and backward paths with min speed
    for(unsigned i = 0; i < fwdPath.size(); ++i) {
        tempComboPathPoint.dist = fwdPath[i].dist;
        if(!m_isReverse) {
            tempComboPathPoint.vel = std::min(fwdPath[i].vel, bwdPath[i].vel);
        }
        else {
            tempComboPathPoint.vel = std::max(fwdPath[i].vel, bwdPath[i].vel);
        }

        m_comboPath.push_back(tempComboPathPoint);
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
    tempFinalPathPoint.time = 0;
    tempFinalPathPoint.dist = 0;
    double firstVel = m_comboPath.front().vel;
    double firstAccel = m_maxAccel * (1 - fabs(firstVel) / m_maxSpeed);
	if(m_isReverse) {
		firstAccel *= -1;;
	}
    tempFinalPathPoint.xPos = m_tempPath.front().xPos;
    tempFinalPathPoint.yPos = m_tempPath.front().yPos;
    tempFinalPathPoint.yaw = m_tempPath.front().yaw;
    tempFinalPathPoint.yawRate = 0;
    m_finalPath.push_back(tempFinalPathPoint);
	double prevVel = firstVel;
    
    // calculate final path
    while(tempFinalPathPoint.time <= m_comboPath.back().time) {
        tempFinalPathPoint.time += 1 / (double)m_sampleRate;
        tempFinalPathPoint.dist = interpolate::interp(comboPathTime, comboPathDist, tempFinalPathPoint.time, false);
        double vel = interpolate::interp(comboPathTime, comboPathVel, tempFinalPathPoint.time, false);
        double accel = (vel - prevVel) * m_sampleRate;
        tempFinalPathPoint.xPos = interpolate::interp(tempPathDist, tempPathXPos, tempFinalPathPoint.dist, false);
        tempFinalPathPoint.yPos = interpolate::interp(tempPathDist, tempPathYPos, tempFinalPathPoint.dist, false);
        double dx = tempFinalPathPoint.xPos - m_finalPath.back().xPos;
        double dy = tempFinalPathPoint.yPos - m_finalPath.back().yPos;
		double dxPerc =  dx / sqrt(dx * dx + dy * dy);
		double dyPerc =  dy / sqrt(dx * dx + dy * dy);
		tempFinalPathPoint.xVel = vel * dxPerc;
		tempFinalPathPoint.yVel = vel * dyPerc;
		tempFinalPathPoint.xAccel = accel * dxPerc;
		tempFinalPathPoint.yAccel = accel * dyPerc;
        tempFinalPathPoint.yaw = interpolate::interp(tempPathDist, tempPathYaw, tempFinalPathPoint.dist, true);
        tempFinalPathPoint.yaw = normalizeToRange::normalizeToRange(tempFinalPathPoint.yaw, -180, 180, true);

        // add vel and accel to first point in final path
        if(m_finalPath.size() == 1) {
			m_finalPath.front().xVel = firstVel * dxPerc;
			m_finalPath.front().yVel = firstVel * dyPerc;
			m_finalPath.front().xAccel = firstAccel * dxPerc;
			m_finalPath.front().yAccel = firstAccel * dyPerc;
        }

        tempFinalPathPoint.yawRate = (tempFinalPathPoint.yaw - m_finalPath.back().yaw) * m_sampleRate;
        m_finalPath.push_back(tempFinalPathPoint);
		prevVel = vel;
    }
}

void SwerveDrivePathGenerator::writePathToCSV() const {
    std::remove(m_pathFilename.c_str());
    
    std::ofstream myFile;
    myFile.open(m_pathFilename);
    myFile << "time (s), xPos (in), yPos (in), yaw (deg), dist (in), xVel (in/s), yVel (in/s), xAccel (in/s^2), yAccel (in/s^2), yawRate (deg/s)\n";
    
    for(unsigned i = 0; i < m_finalPath.size(); ++i) {
        myFile << m_finalPath[i].time << ",";
        myFile << m_finalPath[i].xPos << ",";
        myFile << m_finalPath[i].yPos << ",";
        myFile << m_finalPath[i].yaw << ",";
        myFile << m_finalPath[i].dist << ",";
        myFile << m_finalPath[i].xVel << ",";
		myFile << m_finalPath[i].yVel << ",";
        myFile << m_finalPath[i].xAccel << ",";
		myFile << m_finalPath[i].yAccel << ",";
        myFile << m_finalPath[i].yawRate;
        myFile << "\n";
    }

    myFile.close();
}

void SwerveDrivePathGenerator::writeTempPathToCSV() const {
    std::remove("tempPath.csv");
    
    std::ofstream myFile;
    myFile.open("tempPath.csv");
    myFile << "xPos (in), yPos (in), yaw (deg), vel (in/s), radCurve (in), dist (in)\n";
    
    for(unsigned i = 0; i < m_tempPath.size(); ++i) {
        myFile << m_tempPath[i].xPos << ",";
        myFile << m_tempPath[i].yPos << ",";
		myFile << m_tempPath[i].yaw << ",";
        myFile << m_tempPath[i].vel << ",";
        myFile << m_tempPath[i].radCurve << ",";
        myFile << m_tempPath[i].dist;
        myFile << "\n";
    }

    myFile.close();
}

void SwerveDrivePathGenerator::writeComboPathToCSV() const {
    std::remove("tempComboPath.csv");
    
    std::ofstream myFile;
    myFile.open("tempComboPath.csv");
    myFile << "time (s), xPos (in), yPos (in), yaw (deg), dist (in), vel (in/s), accel (in/s^2), yawRate (deg/s)\n";
    
    for(unsigned i = 0; i < m_comboPath.size(); ++i) {
        myFile << m_comboPath[i].time << ",";
        myFile << m_comboPath[i].xPos << ",";
        myFile << m_comboPath[i].yPos << ",";
        myFile << m_comboPath[i].yaw << ",";
        myFile << m_comboPath[i].dist << ",";
        myFile << m_comboPath[i].vel << ",";
        myFile << m_comboPath[i].accel << ",";
        myFile << m_comboPath[i].yawRate;
        myFile << "\n";
    }

    myFile.close();
}

void SwerveDrivePathGenerator::readWaypointsFromCSV() {
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
    getline(myFile, header); // skip extra delimeters
    setSampleRate(sampleRate);
        
    bool isReverse;
    getline(myFile, header, ',');
    myFile >> isReverse;
    getline(myFile, header); // skip extra delimeters
    setIsReverse(isReverse);
    
    double wheelTrack;
    getline(myFile, header, ',');
    myFile >> wheelTrack;
    getline(myFile, header); // skip extra delimeters
    setWheelTrack(wheelTrack);
        
    double maxSpeed;
    getline(myFile, header, ',');
    myFile >> maxSpeed;
    getline(myFile, header); // skip extra delimeters
    setMaxSpeed(maxSpeed);
    
    double maxAccel;
    getline(myFile, header, ',');
    myFile >> maxAccel;
    getline(myFile, header); // skip extra delimeters
    setMaxAccel(maxAccel);
    
    double maxDeccel;
    getline(myFile, header, ',');
    myFile >> maxDeccel;
    getline(myFile, header); // skip extra delimeters
    setMaxDeccel(maxDeccel);
    
    double maxCentripAccel;
    getline(myFile, header, ',');
    myFile >> maxCentripAccel;
    getline(myFile, header); // skip extra delimeters
    setMaxCentripAccel(maxCentripAccel);
    
    // read data
    getline(myFile, header); // skip blank line
    getline(myFile, header); // skip headers
    
    while(myFile >> tempWaypoint.xPos >> delim
                >> tempWaypoint.yPos >> delim
                >> tempWaypoint.speed >> delim
				>> tempWaypoint.yaw >> delim
                >> tempWaypoint.maxDistThresh) {
        waypoints.push_back(tempWaypoint);
        getline(myFile, header); // skip extra delimeters
    }
    
    // close file
    myFile.close();
    
    // set waypoints
    setWaypoints(waypoints);
}

void SwerveDrivePathGenerator::integratePath(std::vector<pathGenPoint_t> &integratedPath, bool isBackward) {
    pathGenPoint_t tempPathGenPoint;

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

	if(m_isReverse) {
		tempPathGenPoint.vel *= -1;
	}

    integratedPath.push_back(tempPathGenPoint);

    // integrate path
    double accelSpeed;
    double pathSpeed;
    double latSlipSpeed;
    double limitWheelSpeed;
	double radiusCurve;
    unsigned i;
	unsigned j;
	double deltaDist;
	double deltaYaw;
	double leverArm;
    
    if(!isBackward) {
        i = 1;
		
		j = i;
		while(true) {
			if(m_tempPath[j].yaw != std::numeric_limits<double>::infinity()) {
				break;
			}
			j++;
		}
		
		deltaDist = fabs(m_tempPath[j].dist - m_tempPath.front().dist);
		deltaYaw = fabs(normalizeToRange::rangedDifference(m_tempPath[j].yaw - m_tempPath.front().yaw, -180.0, 180.0) * M_PI / 180.0);
		leverArm = sqrt(pow(m_wheelTrack / 2.0, 2) + pow(m_wheelBase / 2.0, 2));
		limitWheelSpeed = fabs(deltaDist * m_maxSpeed / (deltaDist + deltaYaw * leverArm));
    }
    else {
        i = m_tempPath.size() - 2;
		
		j = i;
		while(true) {
			if(m_tempPath[j].yaw != std::numeric_limits<double>::infinity()) {
				break;
			}
			j--;
		}
		
		deltaDist = fabs(m_tempPath[j].dist - m_tempPath.back().dist);
		deltaYaw = fabs(normalizeToRange::rangedDifference(m_tempPath[j].yaw - m_tempPath.back().yaw, -180.0, 180.0) * M_PI / 180.0);
		leverArm = sqrt(pow(m_wheelTrack / 2.0, 2) + pow(m_wheelBase / 2.0, 2));
		limitWheelSpeed = fabs(deltaDist * m_maxSpeed / (deltaDist + deltaYaw * leverArm));
    }
    
    while(((integratedPath.back().dist <= (m_totalPathDist - SWERVE_INTEGRATE_PATH_DIST_STEP)) && !isBackward)
          || ((integratedPath.back().dist >= SWERVE_INTEGRATE_PATH_DIST_STEP) && isBackward)) {
        // increment distance traveled and add to point
        if(!isBackward) {
            tempPathGenPoint.dist += SWERVE_INTEGRATE_PATH_DIST_STEP;
        }
        else {
            tempPathGenPoint.dist -= SWERVE_INTEGRATE_PATH_DIST_STEP;
        }
		
        // get path speed and limit individual wheel speed
        // assume that sample rate is high enough so that temp path points do not need skipped
        pathSpeed = std::numeric_limits<double>::infinity();
        if(!isBackward) {
            if((tempPathGenPoint.dist + SWERVE_INTEGRATE_PATH_DIST_STEP) > m_tempPath[i].dist) {
                pathSpeed = m_tempPath[i].vel;
				
				if(m_tempPath[i].yaw != std::numeric_limits<double>::infinity()) {
					j = i + 1;
					while(true) {
						if(m_tempPath[j].yaw != std::numeric_limits<double>::infinity()) {
							break;
						}
						j++;
					}
					
					deltaDist = fabs(m_tempPath[j].dist - m_tempPath[i].dist);
					deltaYaw = fabs(normalizeToRange::rangedDifference(m_tempPath[j].yaw - m_tempPath[i].yaw, -180.0, 180.0) * M_PI / 180.0);
					leverArm = sqrt(pow(m_wheelTrack / 2.0, 2) + pow(m_wheelBase / 2.0, 2));
					limitWheelSpeed = fabs(deltaDist * m_maxSpeed / (deltaDist + deltaYaw * leverArm));
				}
				
                i++;
            }
            else if(tempPathGenPoint.dist >= m_tempPath[i].dist) {
                pathSpeed = m_tempPath[i].vel;
            }
        }
        else {
            if((tempPathGenPoint.dist - SWERVE_INTEGRATE_PATH_DIST_STEP) < m_tempPath[i].dist) {
                pathSpeed = m_tempPath[i].vel;
				
				if(m_tempPath[i].yaw != std::numeric_limits<double>::infinity()) {
					j = i - 1;
					while(true) {
						if(m_tempPath[j].yaw != std::numeric_limits<double>::infinity()) {
							break;
						}
						j--;
					}
					
					deltaDist = fabs(m_tempPath[j].dist - m_tempPath[i].dist);
					deltaYaw = fabs(normalizeToRange::rangedDifference(m_tempPath[j].yaw - m_tempPath[i].yaw, -180.0, 180.0) * M_PI / 180.0);
					leverArm = sqrt(pow(m_wheelTrack / 2.0, 2) + pow(m_wheelBase / 2.0, 2));
					limitWheelSpeed = fabs(deltaDist * m_maxSpeed / (deltaDist + deltaYaw * leverArm));
				}
				
                i--;
            }
            else if(tempPathGenPoint.dist <= m_tempPath[i].dist) {
                pathSpeed = m_tempPath[i].vel;
            }
        }

        // calculate acceleration speed
        if(!isBackward) {
            accelSpeed = sqrt(pow(tempPathGenPoint.vel, 2) + 2.0 * m_maxAccel * (1 - fabs(tempPathGenPoint.vel) / m_maxSpeed) * SWERVE_INTEGRATE_PATH_DIST_STEP);
        }
        else {
            accelSpeed = sqrt(pow(tempPathGenPoint.vel, 2) - 2.0 * m_maxDeccel * SWERVE_INTEGRATE_PATH_DIST_STEP);
        }

        // hold and limit lateral slip speed through turn
        radiusCurve = std::numeric_limits<double>::infinity();
        if((1 < i) && (i < (m_tempPath.size() - 2))) {
            if(!isBackward) {
                // check if in turn
                if((m_tempPath[i - 1].radCurve == m_tempPath[i].radCurve)
                    && (m_tempPath[i].radCurve != std::numeric_limits<double>::infinity())) {
                    radiusCurve = m_tempPath[i].radCurve;
                }
            }
            else {
                // check if in turn
                if((m_tempPath[i + 1].radCurve == m_tempPath[i].radCurve)
                    && (m_tempPath[i].radCurve != std::numeric_limits<double>::infinity())) {
                    radiusCurve = m_tempPath[i].radCurve;
                }
            }
        }

        latSlipSpeed = sqrt(m_maxCentripAccel * radiusCurve);

        // use minimum speed from all constraints
        tempPathGenPoint.vel = std::min(std::min(std::min(std::min(accelSpeed, pathSpeed), latSlipSpeed), limitWheelSpeed), m_maxSpeed);

        // reverse path direction if needed
        if(m_isReverse) {
        	tempPathGenPoint.vel *= -1;
        }

        // add to path
        integratedPath.push_back(tempPathGenPoint);
    }
    
    if(isBackward) {
        std::reverse(integratedPath.begin(), integratedPath.end());
    }
}

double SwerveDrivePathGenerator::safeACos(double val) const {
    if(val > 1) {
        val = 1;
    }
    else if (val < -1) {
        val = -1;
    }
    
    return acos(val);
}
