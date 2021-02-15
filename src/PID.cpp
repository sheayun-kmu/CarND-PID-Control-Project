#include "PID.h"

#include <iostream>
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
  i_error = 0.0;
  cte_prev = 0.0;
  err_acc = 0.0;
  count = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  std::chrono::time_point<std::chrono::system_clock> curr;
  curr = std::chrono::system_clock::now();
  double dt = std::chrono::duration<double>(
    curr - timestamp
  ).count();
  timestamp = curr;

  if (count == 0) {
    p_error = cte;
    i_error = 0.0;
    d_error = 0.0;
  } else {
    p_error = cte;
    i_error += cte * dt;
    d_error = (cte - cte_prev) / dt;
  }

  /*
  if (i_error > i_error_max) {
    i_error = i_error_max;
  } else if (i_error < -1.0 * i_error_max) {
    i_error = -1.0 * i_error_max;
  }
  */

  cte_prev = cte;
  // accumulate error & count
  err_acc += cte * cte;
  count++;
}

double PID::GetControlValue(void) {
  double pterm = -1.0 * Kp * p_error;
  double iterm = -1.0 * Ki * i_error;
  double dterm = -1.0 * Kd * d_error;
  return pterm + iterm + dterm;
}

double PID::TotalError(void) {
  /**
   * TODO: Calculate and return the total error
   */
  return err_acc / (double) count;
}

void PID::ResetTotalError(void) {
  err_acc = 0.0;
  count = 0;
}
