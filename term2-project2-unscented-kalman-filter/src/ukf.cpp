#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  initialized = false;

  std_a = 2;
  std_psi_dot2 = 0.3;
  std_px = 0.15;
  std_py = 0.15;
  std_rho = 0.3;
  std_phi = 0.03;
  std_rho_dot = 0.3;

  aug_dim = 7;
  x_dim = 5;
  state.x = VectorXd(x_dim);
  state.covariance = MatrixXd(x_dim, x_dim);
  state.timestamp = 0.0;

  weights = VectorXd(2 * aug_dim + 1);
  lambda = 3 - x_dim;
  Xsig_pred = MatrixXd(x_dim, 2 * aug_dim + 1);

  NIS_radar = 0.0;
  NIS_laser = 0.0;
}

void UKF::normalize(double *angle) {
  while (*angle > M_PI) *angle -= 2. * M_PI;
  while (*angle < -M_PI) *angle += 2. * M_PI;
}

UKF::~UKF() {}

VectorXd UKF::get_state_vector() {
  return state.x;
}

void UKF::process(MeasurementPackage mp) {
  if (!initialized) {
    state.x << 1, 1, 1, 1, 0.1;
    state.covariance << 0.15, 0, 0, 0, 0,
                        0, 0.15, 0, 0, 0,
                        0, 0, 1, 0, 0,
                        0, 0, 0, 1, 0,
                        0, 0, 0, 0, 1;
    state.timestamp = mp.timestamp_;

    if (mp.sensor_type_ == MeasurementPackage::LASER) {
      state.x(0) = mp.raw_measurements_(0);
      state.x(1) = mp.raw_measurements_(1);
    } else if (mp.sensor_type_ == MeasurementPackage::RADAR) {
      float ro = mp.raw_measurements_(0);
      float phi = mp.raw_measurements_(1);
      float ro_dot = mp.raw_measurements_(2);
      state.x(0) = ro * cos(phi);
      state.x(1) = ro * sin(phi);
    }
    initialized = true;
    return;
  }

  float dt = (mp.timestamp_ - state.timestamp) / 1000000.0;	//seconds
  state.timestamp = mp.timestamp_;

  predict(dt);
  if (mp.sensor_type_ == MeasurementPackage::LASER) {
    update_lidar(mp);
  } else if (mp.sensor_type_ == MeasurementPackage::RADAR) {
    update_radar(mp);
  }
}

void UKF::predict(double dt) {
  int length = 2 * aug_dim + 1;
  MatrixXd Xsig = MatrixXd(x_dim, 2 * x_dim + 1);
  MatrixXd A = state.covariance.llt().matrixL();

  lambda = 3 - x_dim;
  Xsig.col(0) = state.x;

  for(int i = 0; i < x_dim; i++) {
    Xsig.col(i + 1) = state.x + sqrt(lambda + x_dim) * A.col(i);
    Xsig.col(i + 1 + x_dim) = state.x - sqrt(lambda + x_dim) * A.col(i);
  }

  VectorXd x_aug = VectorXd(aug_dim);
  MatrixXd P_aug = MatrixXd(aug_dim, aug_dim);
  MatrixXd Xsig_aug = MatrixXd(aug_dim, length);

  lambda = 3 - aug_dim;

  x_aug.head(5) = state.x;
  x_aug(5) = 0;
  x_aug(6) = 0;

  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = state.covariance;
  P_aug(5, 5) = pow(std_a, 2);
  P_aug(6, 6) = pow(std_psi_dot2, 2);

  MatrixXd L = P_aug.llt().matrixL();
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i < aug_dim; i++) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda + aug_dim) * L.col(i);
    Xsig_aug.col(i + 1 + aug_dim) = x_aug - sqrt(lambda + aug_dim) * L.col(i);
  }

  for(int i = 0; i < length; i++) {
    double p_x =          Xsig_aug(0, i);
    double p_y =          Xsig_aug(1, i);
    double v =            Xsig_aug(2, i);
    double psi =          Xsig_aug(3, i);
    double psi_dot =      Xsig_aug(4, i);
    double nu_a =         Xsig_aug(5, i);
    double nu_psi_dot2 =  Xsig_aug(6, i);
    double px_p, py_p;

    if(fabs(psi_dot) > 0.001) {
      px_p = p_x + v / psi_dot * (sin(psi + psi_dot * dt) - sin(psi));
      py_p = p_y + v / psi_dot * (cos(psi) - cos(psi + psi_dot * dt));
    } else {
      px_p = p_x + v * dt * cos(psi);
      py_p = p_y + v * dt * sin(psi);
    }

    double v_p = v;
    double psi_p = psi + psi_dot * dt;
    double psi_dot_p = psi_dot;

    px_p += 0.5 * nu_a * dt * dt * cos(psi);
    py_p += 0.5 * nu_a * dt * dt * sin(psi);
    v_p += nu_a * dt;

    psi_p += 0.5 * nu_psi_dot2 * dt * dt;
    psi_dot_p += nu_psi_dot2 * dt;

    Xsig_pred(0, i) = px_p;
    Xsig_pred(1, i) = py_p;
    Xsig_pred(2, i) = v_p;
    Xsig_pred(3, i) = psi_p;
    Xsig_pred(4, i) = psi_dot_p;
  }

  for (int i = 0; i < length; i++) {  //2n+1 weights
    if (i == 0) {
      weights(0) = lambda / (lambda + aug_dim);
      continue;
    }

    weights(i) = 0.5 / (aug_dim + lambda);
  }

  state.x.fill(0.0);
  for (int i = 0; i < length; i++) {
    state.x += weights(i) * Xsig_pred.col(i);
  }

  state.covariance.fill(0.0);
  for (int i = 0; i < length; i++) {
    VectorXd x_diff = Xsig_pred.col(i) - state.x;
    normalize(&x_diff(3));

    state.covariance += weights(i) * x_diff * x_diff.transpose();
  }

}

void UKF::update_lidar(MeasurementPackage mp) {
  int length = 2 * aug_dim + 1;
  VectorXd z = mp.raw_measurements_;
  int n_z = 2;

  MatrixXd Zsig = MatrixXd(n_z, length);

  for (int i = 0; i < length; i++) {
    double p_x = Xsig_pred(0, i);
    double p_y = Xsig_pred(1, i);

    Zsig(0, i) = p_x;
    Zsig(1, i) = p_y;
  }

  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < length; i++) {
    z_pred += weights(i) * Zsig.col(i);
  }

  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < length; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S += weights(i) * z_diff * z_diff.transpose();
  }

  MatrixXd R = MatrixXd(n_z, n_z);
  R << pow(std_px, 2), 0,
       0, pow(std_py, 2);
  S += R;

  MatrixXd Tc = MatrixXd(x_dim, n_z);

  Tc.fill(0.0);
  for (int i = 0; i < length; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    VectorXd x_diff = Xsig_pred.col(i) - state.x;

    Tc += weights(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;
  NIS_laser = z_diff.transpose() * S.inverse() * z_diff;
  state.x += K * z_diff;
  state.covariance += - K * S * K.transpose();
}

void UKF::update_radar(MeasurementPackage mp) {
  int length = 2 * aug_dim + 1;
  VectorXd z = mp.raw_measurements_;
  int n_z = 3;

  MatrixXd Zsig = MatrixXd(n_z, length);

  for (int i = 0; i < length; i++) {

    double p_x = Xsig_pred(0, i);
    double p_y = Xsig_pred(1, i);
    double v   = Xsig_pred(2, i);
    double yaw = Xsig_pred(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    Zsig(0, i) = sqrt(pow(p_x, 2) + pow(p_y, 2));
    Zsig(1, i) = atan2(p_y, p_x);
    Zsig(2, i) = (p_x * v1 + p_y*v2) / Zsig(0, i);
  }

  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < length; i++) {
    z_pred += weights(i) * Zsig.col(i);
  }

  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < length; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    normalize(&z_diff(1));

    S += weights(i) * z_diff * z_diff.transpose();
  }

  MatrixXd R = MatrixXd(n_z, n_z);
  R << pow(std_rho, 2), 0, 0,
       0, pow(std_phi, 2), 0,
       0, 0, pow(std_rho_dot, 2);
  S += R;

  MatrixXd Tc = MatrixXd(x_dim, n_z);

  Tc.fill(0.0);
  for (int i = 0; i < length; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    VectorXd x_diff = Xsig_pred.col(i) - state.x;

    normalize(&z_diff(1));
    normalize(&x_diff(3));

    Tc += weights(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;
  normalize(&z_diff(1));

  NIS_radar = z_diff.transpose() * S.inverse() * z_diff;
  state.x += K * z_diff;
  state.covariance += - K * S * K.transpose();
}