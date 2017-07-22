#ifndef PID_H
#define PID_H

#include<vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients for steering angle control
  */ 
  double Kp;
  double Ki;
  double Kd;

  // Coefficients for throttle control based on CTE
  double Kd_throttle ;
  double Kp_throttle ;
  double Ki_throttle ;

  // Keep track of measurements
  long meas ;
  double best_twiddle_error;
  double twiddle_error ;
  int flag;

  // update parameters
  std::vector<double> dparams;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double K_p, double K_i, double K_d, double K_p_throttle, double K_i_throttle, double K_d_throttle);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Return the Throttle Value based on the error
   */
  double ThrottleUpdate();

  void Twiddle() ;
};

#endif /* PID_H */
