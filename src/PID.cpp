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

