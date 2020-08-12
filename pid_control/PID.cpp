#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
	/**
	 * TODO: Initialize PID coefficients (and errors, if needed)
	 */
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	previous_cte = 0;
	i_error = 0;

}

void PID::UpdateError(double cte) {
	/**
	 * TODO: Update PID errors based on cte.
	 */
	p_error = cte;
	i_error = cte + i_error;
	d_error = cte - previous_cte;
	previous_cte = cte;

}

double PID::TotalError() {
	/**
	 * TODO: Calculate and return the total error
	 */
	return Kp * p_error + Ki * i_error + Kd * d_error;;  // TODO: Add your total error calc here!
}