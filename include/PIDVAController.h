//////////////////////////////////////////////////////////////////////
// PIDVA controller
//////////////////////////////////////////////////////////////////////
#pragma once

class PIDVAController {
public:
	PIDVAController();
	PIDVAController(
		const double &kp,
		const double &ki,
		const double &kd,
		const double &kv,
		const double &ka);
	~PIDVAController();
	
	double getKp() const;
	double getKi() const;
	double getKd() const;
	double getKv() const;
	double getKa() const;
	double getRangeMin() const;
	double getRangeMax() const;
	bool getIsContinous() const;
	double getIZone() const;
	double getIErrorLim() const;
	double getTargetZone() const;
	double getTargetZoneDebounce() const;
	
	void setKp(const double &kp);
	void setKi(const double &ki);
	void setKd(const double &kd);
	void setKv(const double &kv);
	void setKa(const double &ka);
	
	//////////////////////////////////////////////////////////////////////
    // @brief set to true if reference wraps around from rangeMax to
	//        rangeMin such as angle range of +/-180 deg
    //////////////////////////////////////////////////////////////////////
	void setIsContinous(
		const bool &isContinous,
		const double &rangeMin,
		const double &rangeMax);
	
	//////////////////////////////////////////////////////////////////////
    // @brief integral term only takes effect when measurement is within
	//        +/-iZone of reference to prevent integral windup
    //////////////////////////////////////////////////////////////////////
	void setIZone(const double &iZone);
	
	//////////////////////////////////////////////////////////////////////
    // @brief integral error is limited to +/-iErrorLim to prevent integral
	//        windup
    //////////////////////////////////////////////////////////////////////
	void setIErrorLim(const double &iErrorLim);
	
	void setTargetZone(const double &targetZone);
	void setTargetZoneDebounce(const double &targetZoneDebounce);

	//////////////////////////////////////////////////////////////////////
    // @brief update control signal from controller.  must be called
	//        periodically for time derivatives and integrals to work
	//        properly.
    //////////////////////////////////////////////////////////////////////
	void update(
		double &refP,
		const double &refV,
		const double &refA,
		const double &measP,
		const double &time,
		double &cntrl);
	
	//////////////////////////////////////////////////////////////////////
    // @brief is measurement on target with reference.  isOnTarget is set
	//        to TRUE, control is set to zero, error terms are reset when
	//        error is within +/-targetZone for m_targetZoneDebounce period
	//        time.
    //////////////////////////////////////////////////////////////////////
	bool isOnTarget();
	
	//////////////////////////////////////////////////////////////////////
    // @brief reset error terms of controller
    //////////////////////////////////////////////////////////////////////
	void reset(const double &time);
	
private:
	double m_kp;					// feed-back proportional gain
	double m_ki;					// feed-back integral gain
	double m_kd;					// feed-back derivative gain
	double m_kv;					// feed-forward velocity gain
	double m_ka;					// feed-forward accel gain
	double m_rangeMin;				// min range (units)
	double m_rangeMax;              // max range (units)
	bool m_isContinous;             // flag indicating wraparound
	double m_iZone;					// i term is only enabled when error is within this zone
	double m_iErrorLim;				// i error saturation limit to prevent integral windup (units * s)
	double m_targetZone;			// target zone (units)
	double m_targetZoneDebounce;	// amount of time that measurement must be in target zone to call on target (s)
	double m_timeOld;				// timestamp from last update step (s)
	double m_errorOld;				// error from last update step (units)
	double m_iErrorOld;				// i error from last update step (units * s)
	bool m_isOnTarget;				// flag indicating is on target
	double m_isOnTargetStartTime;	// timestamp when first on target (s) 
	bool m_onTargetFirstTime;		// flag indicating if on target for first time
};
