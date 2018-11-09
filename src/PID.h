#ifndef PID_H
#define PID_H
#include <vector>
#include <limits>
class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  /*
  * Parameters for twiddle
  */
  bool parameter_twiddled;
  bool reset;
  bool tried_add;
  bool tried_substract;
  //vector of change
  std::vector<double> dp;
  
  int step;
  int parameter_index;
  // number of steps to allow simulator to run forward, then to evaluate cte
  int n_forward_steps, n_eval_steps;
  // error tracker
  double total_error, best_error;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
  * Find the optimal Kp, Ki, Kd values
  */
  void Twiddle(double cte, double tol, int max_steps);
};

#endif /* PID_H */
