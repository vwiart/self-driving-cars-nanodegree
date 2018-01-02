#include "PID.h"

using namespace std;

PID::PID(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;
}

PID::~PID() {}

void PID::UpdateError(double cte) {
    double prev_cte = p_error;
    p_error = cte;
    d_error = cte - prev_cte;
    i_error += cte;
}

double PID::TotalError() {
    return -Kp * p_error - Kd * d_error - Ki * i_error;
}

