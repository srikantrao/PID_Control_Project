#include "PID.h"
#include<iostream>
#include<math.h>
#include<vector>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID():best_twiddle_error(100000.0) {}

PID::~PID() {}

void PID::Init(double K_p, double K_i, double K_d, double K_p_throttle, double K_i_throttle,
		double K_d_throttle)
{
	this->Kp = K_p ;
	this->Kd = K_d ;
	this->Ki = K_i ;
	this->Kp_throttle = K_p_throttle ;
	this->Kd_throttle = K_d_throttle ;
	this->Ki_throttle = K_i_throttle ;
	this->meas = 0 ;
	this->twiddle_error = 0;
	this->flag = 0;

	// Set the errors to zero
	this->d_error = 0;
	this->i_error = 0;
	this->p_error = 0;

	// Set update parameters

	this->dparams = {0.05, 0.001, 0.25, 0.05, 0.001, 0.5};
 }

void PID::UpdateError(double cte) {
	// Determine the coefficients
	d_error = cte - p_error ; // You need delta_cte
	p_error = cte ;
	i_error += cte ;
	twiddle_error += fabs(cte) ; // calculate the absolute error
	meas++ ;
}

double PID::TotalError() {


	double alpha = (-Kp * p_error -Kd * d_error - Ki * i_error);

	// Need to make sure it does not exceed maximum limit
	if(alpha > 1.0 ) alpha = 1.0 ;

	else if(alpha < -1.0 ) alpha = -1.0;

	return alpha ;
}

double PID::ThrottleUpdate() {
	double beta = 0.7 - Kp_throttle * fabs(p_error) - Kd_throttle * fabs(d_error)  - Ki_throttle * fabs(i_error);

	if(beta < 0.1 ) beta = 0.1 ;
	else if(beta >  0.7 ) beta = 0.7 ;

	return beta ;
}

void PID::Twiddle(){

	// One whole lap has been completed
	if(meas >= 700) {

		// Create a new set of vectors
		vector<double> params = {Kp, Ki, Kd, Kp_throttle, Ki_throttle, Kd_throttle } ;

		//Increment the params
		for(unsigned int i =0; i < params.size(); i++)
			params[i] += dparams[i] ;
		flag = 1;

		if(twiddle_error < best_twiddle_error && flag ==1){
			best_twiddle_error = twiddle_error ;
			for(unsigned int i =0; i < dparams.size() ; i++)
				dparams[i] = dparams[i] * 1.1 ;
		}
		if (twiddle_error > best_twiddle_error && flag ==1){
			for(unsigned int i =0; i < params.size(); i++)
				params[i] -= 2*dparams[i] ;
				flag = 2;

		}
		if(twiddle_error < best_twiddle_error && flag == 2){
			best_twiddle_error = twiddle_error ;
			//Increment the params
			for(unsigned int i =0; i < params.size(); i++)
				params[i] += dparams[i] ;

		}

		if(twiddle_error > best_twiddle_error && flag == 2){
			for(unsigned int i =0; i < params.size(); i++){
				params[i] += dparams[i] ;
				dparams[i] *= 0.9 ;
				flag = 0;
			}

		}

		cout << "The updated parameters have been updated " << endl ;
		for(unsigned int i =0; i < params.size(); i ++)
			cout << "Parameter " << i << "  : " << params[i] << endl;
		Init(params[0], params[1], params[2], params[3], params[4], params[5]) ;
	}

}
