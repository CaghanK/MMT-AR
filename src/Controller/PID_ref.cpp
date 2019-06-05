class PID {
	public:
		PID();
		PID(double _kp, double _ki, double _kd);
		void setTarget(double _target);
		void setInput(double _input);
		void setTunings(double _kp, double _ki, double _kd);
        void setOutputLimits(double _min, double _max);
		double compute();
		double getKp();
		double getKi();
		double getKd();
        double getTarget();
        double getInput();
		void setActive(bool _active);
		
	private:
		void reset();
		double kp, ki, kd;
		double input, target, output;
		double ITotal, prevInput, prevError;
        double minLimit, maxLimit;
		bool active;
		ros::Time prevTime;
};
 
PID::PID(double _kp, double _ki, double _kd) {
	kp = _kp;
	ki = _ki;
	kd = _kd;
	reset();
    output = 0;
    input = 0;
    
    setOutputLimits(0, 255);
	
	//ROS_DEBUG("Kp %.2f; Ki %.2f; Kd %.2f", kp, ki, kd);
}
 
PID::PID() {
	reset();
}
 
void PID::setTarget(double _target) {
	target = _target;
}
 
void PID::setInput(double _input) {
	input = _input;
}
 
void PID::setTunings(double _kp, double _ki, double _kd) {
	kp = _kp;
	ki = _ki;
	kd = _kd;
    
    reset();
};
 
void PID::setOutputLimits(double _min, double _max) {
    if (_min > _max) return;
    
    minLimit = _min;
    maxLimit = _max;
}
 
double PID::compute() {
	ros::Time now = ros::Time::now();
	ros::Duration change = now - prevTime;
	
	double error = target - input;
	ITotal += error * ki;
    ITotal = std::min(ITotal, maxLimit);
	ITotal = std::max(ITotal, minLimit);    
	double dValue = kd * (error - prevError)/change.toSec();
	
	/* do the full calculation */
	output = kp * error + ITotal + dValue;
    
    /* clamp output to bounds */
    output = std::min(output, maxLimit);
	output = std::max(output, minLimit);  
	
	/* required values for next round */
	prevTime = now;
	prevInput = input;
    
    /* debug some PID settings */
    //ROS_DEBUG("P %.2f; I %.2f; D %.2f", error, ITotal, dValue);
	
	return output;
};
 
double PID::getKp() {
	return kp;
}
 
double PID::getKi() {
	return ki;
}
 
double PID::getKd() {
	return kd;
}
 
double PID::getTarget() {
    return target;
}
 
double PID::getInput() {
    return input;
}
 
void PID::setActive(bool _active) {
	if (!active && _active)
		reset();
	active = _active;
};
 
void PID::reset() {
	ITotal = output;
	prevInput = 0;
	prevError = 0;
    
    ITotal = std::min(ITotal, maxLimit);
	ITotal = std::max(ITotal, minLimit); 
};