#ifndef PID_H
#define PID_H

#include <chrono>

class PID {
 public:
  static const constexpr double i_error_max = 5.0 * 0.03;
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Get the current control value.
   * @output The control value calculated by PID
   */
  double GetControlValue(void);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError(void);

  /**
   * Reset the total PID error.
   */
  void ResetTotalError(void);

  /**
   * Prints out messages for debugging.
   */
  void DebugDisplay(void);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double cte_prev;
  double err_acc;

  /**
   * PID terms
   */
  double pterm;
  double iterm;
  double dterm;

  /**
   * PID Coefficients
   */
  double Kp;
  double Ki;
  double Kd;

  int count;

  /**
   * timestamp
   */
  std::chrono::time_point<std::chrono::system_clock> timestamp;
};

#endif  // PID_H