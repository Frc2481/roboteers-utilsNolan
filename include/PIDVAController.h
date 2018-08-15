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
		const double &ka,
		const double &kv);
	~PIDVAController();
	
	double getKp();
	double getKi();
	double getKd();
	double getKa();
	double getKv();
	double getRangeMin();
	double getRangeMax();
	bool getIsContinous();
	double getIZone();
	double getIErrorLim();
	double getTargetZone();
	double getTargetZoneDebounce();
	
	void setKp(const double &kp);
	void setKi(const double &ki);
	void setKd(const double &kd);
	void setKa(const double &ka);
	void setKv(const double &kv);
	void setRangeMin(const double &rangeMin);
	void setRangeMax(const double &rangeMax);
	
	//////////////////////////////////////////////////////////////////////
    // @brief set to true if reference wraps around from rangeMax to
	//        rangeMin such as angle range of +/-180 deg
    //////////////////////////////////////////////////////////////////////
	void setIsContinous(const bool &isContinous);
	
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
		const double &refP,
		const double &refV,
		const double &refA,
		const double &measP,
		const double &time,
		double cntrl);
	
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
	void reset();
	
private:
	double m_kp;
	double m_ki;
	double m_kd;
	double m_ka;
	double m_kv;
	double m_rangeMin;
	double m_rangeMax;
	bool m_isContinous;
	double m_iZone;
	double m_iErrorLim;
	double m_targetZone;
	double m_targetZoneDebounce;
	double m_timeOld;
	double m_errorOld;
	bool m_isOnTarget;
	unsigned m_isOnTargetCnt;
};