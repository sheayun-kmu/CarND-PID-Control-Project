#include <iostream>
#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(const std::string& name, double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  id = name;
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
   * Update PID errors based on cte.
   */
  std::chrono::time_point<std::chrono::system_clock> curr;
  curr = std::chrono::system_clock::now();
  // Calculate time elapsed between the previous error update
  // and the current one (for determining i_error & d_error).
  double dt = std::chrono::duration<double>(
    curr - timestamp
  ).count();
  timestamp = curr;

  // If the update is the very first one, we cannot determine the
  // i_error and d_error since no timing information is given
  // - simply initialize them to zeros.
  if (count == 0) {
    p_error = cte;
    i_error = 0.0;
    d_error = 0.0;
  } else {
    p_error = cte;
    i_error += cte * dt;
    d_error = (cte - cte_prev) / dt;
  }

  // Measures to prevent integral windup - cutoff at a predetermined threshold.
  if (i_error > i_error_max) {
    i_error = i_error_max;
  } else if (i_error < -1.0 * i_error_max) {
    i_error = -1.0 * i_error_max;
  }

  cte_prev = cte;
  // Accumulate error & count (for calculating MSE).
  err_acc += cte * cte;
  count++;
}

double PID::GetControlValue(void) {
  /**
   * Provide control value by simply adding the three terms.
   */
  pterm = -1.0 * Kp * p_error;
  iterm = -1.0 * Ki * i_error;
  dterm = -1.0 * Kd * d_error;
  return pterm + iterm + dterm;
}

double PID::TotalError(void) {
  /**
   * Calculate and return the total error
   */
  return err_acc / (double) count;
}

void PID::ResetTotalError(void) {
  /**
   * Reset the accumulated error value and the count.
   * NOTE: Because of the counter reset, the next error update will
   *       use zero (not precise) values for i_error and d_error.
   */
  err_acc = 0.0;
  count = 0;
}

void PID::DebugDisplay(void) {
  std::cout << "================================================" << std::endl;
  std::cout << '[' << id << "] (" << count << ')' << std::endl;
  std::cout << "\tPE: " << p_error << ", IE: " << i_error
            << ", DE: " << d_error << std::endl;
  std::cout << "\tP: " << pterm << ", I: " << iterm
            << ", D: " << dterm << std::endl;
  std::cout << "================================================" << std::endl;
}
