#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct State {
  VectorXd x;
  MatrixXd covariance;
  long long timestamp;
};

class UKF {
public:
  UKF();
  virtual ~UKF();
  void process(MeasurementPackage mp);
  VectorXd get_state_vector();
private:
  void normalize(double *angle);
  void predict(double dt);
  void update_lidar(MeasurementPackage mp);
  void update_radar(MeasurementPackage mp);
  bool initialized;
  State state;
  MatrixXd Xsig_pred;
  double std_a, std_psi_dot2, std_px, std_py, std_rho, std_phi, std_rho_dot ;
  VectorXd weights;
  int x_dim;
  int aug_dim;
  double lambda;
  double NIS_radar;
  double NIS_laser;
};

#endif
