#include "PID.h"
#include <cmath>
#include <iostream>
#include <numeric>

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
  // Twiddling parameters
  this->parameter_twiddled = false;
  this->reset = false;
  this->tried_add = false;
  this->tried_substract = false;
  this->dp = {0.1*Kp, 0.1*Kd, 0.1*Ki};
  this->step = 1;
  this->parameter_index = 2; 
  this->n_forward_steps = 10;
  this->n_eval_steps = 1000;
  this->total_error = 0;
  this->best_error = std::numeric_limits<double>::max();
}

void PID::UpdateError(double cte) {
  double prev_cte = this->p_error;
  this->d_error = cte - prev_cte;
  this->p_error = cte;
  double prev_total_cte = this->i_error;
  this->i_error = prev_total_cte + cte;
}

double PID::TotalError() {
  return -this->Kp * this->p_error - this->Kd * this->d_error - this->Ki * this->i_error;
}

void PID::Twiddle(double cte, double tol, int max_steps){
  vector<double> p;
  p = {Kp, Kd, Ki};
  //dp = {0.1*Kp, 0.1*Kd, 0.1*Ki};
  // allow vehicle to move forward 
  if (step % (n_forward_steps + n_eval_steps) > n_forward_steps){
    total_error += pow(cte,2);
  }

  //cout << "total error" << total_error << endl;
  //run the simulator forward for N steps (n_forward_steps + n_eval_steps)
  if(step % (n_forward_steps + n_eval_steps) == 0) {
    cout << "step: " << step << endl;
    cout << "total error: " << total_error << endl;
    cout << "best error: " << best_error << endl;
    //find a successful improvement
    if(total_error < best_error){
      best_error = total_error;
      //since first time total_error will be smaller than maximum number, we don't want to change dp
      if(step != n_forward_steps + n_eval_steps){
        dp[parameter_index] *= 1.1;
      }
      //go to next parameter
      parameter_index = (parameter_index + 1) % 3;
      //reset the flag 
      tried_add = false;
      tried_substract = false;
    }
    //if haven't tried either adding or substracting
    if(!tried_add && !tried_substract){
      //try add dp for parameter at parameter_index
      p[parameter_index] += dp[parameter_index];
      //add once is enough, use a flag to track it 
      tried_add = true;
    }
    //if tried adding but did not reduce total error 
    else if(tried_add && !tried_substract){
      //try minus dp
      p[parameter_index] -= 2*dp[parameter_index];
      //substract once is enough, use a flag to track it 
      tried_substract = true;
    }
    //tried both add and substract but did not get a better total error
    else{
      //set to original value
      p[parameter_index] += dp[parameter_index];
      //reduce dp
      dp[parameter_index] *= 0.9;
      //go to next parameter
      parameter_index = (parameter_index + 1) % 3;
      //reset the tracker 
      tried_add = false;
      tried_substract = false;
    }
    total_error = 0;
    cout << "new parameters" << endl;
    cout << "P: " << p[0] << ", I: " << p[2] << ", D: " << p[1] << endl;
  }//end of if
  step++;
  if(dp[0]+dp[1]+dp[2]<tol && step > max_steps){
    Kp = p[0];
    Ki = p[2];
    Kd = p[1];
    parameter_twiddled = true;
    reset = true;}
}

