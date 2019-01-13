#include "PIDVAController.h"
#include <cmath>
#include "NormalizeToRange.h"

PIDVAController::PIDVAController()
    : m_kp(0),
	m_ki(0),
	m_kd(0),
	m_kv(0),
    m_ka(0),
	m_rangeMin(0),
	m_rangeMax(0),
	m_isContinous(false),
	m_iZone(0),
	m_iErrorLim(0),
	m_targetZone(0),
	m_targetZoneDebounce(0),
	m_timeOld(0),
	m_errorOld(0),
    m_iErrorOld(0),
	m_isOnTarget(false),
	m_isOnTargetStartTime(0),
    m_onTargetFirstTime(true) {
}

PIDVAController::PIDVAController(
		double kp,
		double ki,
		double kd,
		double kv,
		double ka) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_kv = kv;
    m_ka = ka;
    m_rangeMin = 0;
	m_rangeMax = 0;
	m_isContinous = false;
	m_iZone = 0;
	m_iErrorLim = 0;
	m_targetZone = 0;
	m_targetZoneDebounce = 0;
	m_timeOld = 0;
	m_errorOld = 0;
    m_iErrorOld = 0;
	m_isOnTarget = false;
	m_isOnTargetStartTime = 0;
    m_onTargetFirstTime = true;
}

PIDVAController::~PIDVAController() {
}

void PIDVAController::setKp(double kp) {
    m_kp = kp;
}

void PIDVAController::setKi(double ki) {
    m_ki = ki;
}

void PIDVAController::setKd(double kd) {
    m_kd = kd;
}

void PIDVAController::setKv(double kv) {
    m_kv = kv;
}

void PIDVAController::setKa(double ka) {
    m_ka = ka;
}

void PIDVAController::setIsContinous(
    bool isContinous,
    double rangeMin,
    double rangeMax) {
    
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

void PIDVAController::setIZone(double iZone) {
    m_iZone = fabs(iZone);
}

void PIDVAController::setIErrorLim(double iErrorLim) {
    m_iErrorLim = fabs(iErrorLim);
}

void PIDVAController::setTargetZone(double targetZone) {
    m_targetZone = fabs(targetZone);
}

void PIDVAController::setTargetZoneDebounce(double targetZoneDebounce) {
    m_targetZoneDebounce = targetZoneDebounce;
}

void PIDVAController::update(
    double refP,
    double refV,
    double refA,
    double measP,
    double time,
    double &cntrl) {

    // skip update if time didn't change
    if(time == m_timeOld) {
        return;
    }

    // check reference wraparound
    if(m_isContinous) {
        refP = normalizeToRange::normalizeToRange(refP, m_rangeMin, m_rangeMax, true);
    }

    // calculate error
    double error = refP - measP;
    
    // check wraparound
    if(m_isContinous) {
        normalizeToRange::rangedDifference(error, m_rangeMin, m_rangeMax);
    }

    // calculate i error
    double iError = error * (time - m_timeOld) + m_iErrorOld;

    // limit i error
    if(iError > m_iErrorLim) {
        iError = m_iErrorLim;
    }
    else if(iError < -m_iErrorLim) {
        iError = -m_iErrorLim;
    }

    // only apply i term if in i zone
    if(fabs(error) > m_iZone) {
        iError = 0;
    }

    // calculate d error
    double dError = (error - m_errorOld) / (time - m_timeOld);
    
    // calculate control signal
    cntrl = m_kp * error
        + m_ki * iError
        + m_kd * dError
        + m_kv * refV
        + m_ka * refA;

    // check if on target
    if(fabs(error) < m_targetZone) {
        // check if on target for first time
        if(m_onTargetFirstTime) {
            m_isOnTargetStartTime = time;
            m_onTargetFirstTime = false;
        }

        // debounce on target
        if((time - m_isOnTargetStartTime) > m_targetZoneDebounce) {
            m_isOnTarget = true;
            reset(time);
            cntrl = 0;
            return;
        }
    }
    else {
        m_isOnTarget = false;
        m_onTargetFirstTime = true;
    }

    // save controller states for next loop
    m_timeOld = time;
	m_errorOld = error;
    m_iErrorOld = iError;
}

bool PIDVAController::isOnTarget() const {
    return m_isOnTarget;
}

void PIDVAController::reset(double time) {
	m_timeOld = time;
	m_errorOld = 0;
    m_iErrorOld = 0;
	m_isOnTarget = false;
	m_isOnTargetStartTime = 0;
    m_onTargetFirstTime = true;
}
