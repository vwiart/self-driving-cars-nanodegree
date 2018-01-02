#ifndef PID_H
#define PID_H

class PID {
public:
  // errors
  double p_error, i_error,d_error;

  // coefficients
  double Kp, Ki, Kd;

  PID(double Kp, double Ki, double Kd);
  virtual ~PID();
  void UpdateError(double cte);
  double TotalError();
};

#endif
