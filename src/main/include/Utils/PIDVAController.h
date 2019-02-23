#ifndef PIDVA_CONTROLLER_H
#define PIDVA_CONTROLLER_H

class PIDVAController {
public:
	PIDVAController();
	PIDVAController(
		double kp,
		double ki,
		double kd,
		double kv,
		double ka);
	~PIDVAController();
	
	void setKp(double kp);
	void setKi(double ki);
	void setKd(double kd);
	void setKv(double kv);
	void setKa(double ka);
	
	//////////////////////////////////////////////////////////////////////
    // @brief set to true if reference wraps around from rangeMax to
	//        rangeMin such as angle range of +/-180 deg
    //////////////////////////////////////////////////////////////////////
	void setIsContinous(
		bool isContinous,
		double rangeMin,
		double rangeMax);
	
	//////////////////////////////////////////////////////////////////////
    // @brief integral term only takes effect when measurement is within
	//        +/-iZone of reference to prevent integral windup
    //////////////////////////////////////////////////////////////////////
	void setIZone(double iZone);
	
	//////////////////////////////////////////////////////////////////////
    // @brief integral error is limited to +/-iErrorLim to prevent integral
	//        windup
    //////////////////////////////////////////////////////////////////////
	void setIErrorLim(double iErrorLim);
	
	void setTargetZone(double targetZone);
	void setTargetZoneDebounce(double targetZoneDebounce);

	//////////////////////////////////////////////////////////////////////
    // @brief update control signal from controller.  must be called
	//        periodically for time derivatives and integrals to work
	//        properly.
    //////////////////////////////////////////////////////////////////////
	void update(
		double refP,
		double refV,
		double refA,
		double measP,
		double time,
		double &cntrl);
	
	//////////////////////////////////////////////////////////////////////
    // @brief is measurement on target with reference.  isOnTarget is set
	//        to TRUE, control is set to zero, error terms are reset when
	//        error is within +/-targetZone for m_targetZoneDebounce period
	//        time.
    //////////////////////////////////////////////////////////////////////
	bool isOnTarget() const;
	
	//////////////////////////////////////////////////////////////////////
    // @brief reset error terms of controller
    //////////////////////////////////////////////////////////////////////
	void reset(double time);
	
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

#endif // PIDVA_CONTROLLER_H
