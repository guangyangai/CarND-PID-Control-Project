#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->p_error = 0;
  this->d_error = 0;
  this->i_error = 0;
}

void PID::UpdateError(double cte) {
  double prev_cte = this->p_error;
  this->d_error = cte - prev_cte;
  this->p_error = cte;
  double prev_total_cte = this->i_error;
  this->i_error = prev_total_cte + cte;
}

double PID::TotalError() {
  return this->i_error;
}

void PID::Twiddle(double cte, double speed, double angle, int steps, double tol){
  p = {Kp, Kd, Ki};
  dp = {0.1*Kp, 0.1*Kd, 0.1*Ki};
  //run the simulator forward for N steps 
  best_err = run(robot, p);
  while (sum(dp) > tol) {
    for(unsigned i=0; i<p.size(); i++){
      p[i] += dp[i];
      //run the simulator forward for N steps
      new_err = run(robot, p);
      if(new_err < best_err){
        best_error = new_err;
        dp[i]*=1.1;
      }else{
        //tru minus dp
        p[i] -= 2*dp[i];
        //forward the simulator N times 
        new_err = run(robot, p);
        if(new_err < best_err){
          best_error = new_err;
          dp[i]*=1.1;
        }else{
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }//end of previous else
    }//end of for
  }//end of while
}

