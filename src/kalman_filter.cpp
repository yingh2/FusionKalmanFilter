#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::generateUpdate(VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  generateUpdate(y);
}

const double PI = 3.141592653589793238463;

VectorXd KalmanFilter::calculateZPred(const VectorXd &x) {
  float rho = x[0] * x[0] + x[1] * x[1];
  rho = sqrt(rho);
  float phi = calculatePhi(x[0], x[1]);
  float rhodot = 0.0;
  if (fabs(rho) > 0.001) {
    rhodot = (x[0] * x[2] + x[1] * x[3]) / rho;
  }
  VectorXd z_pred(3);
  z_pred << rho, phi, rhodot;
  return z_pred;
}

float KalmanFilter::normalizePhi(float phi) {
  while (phi >= PI) {
    phi -= (2 * PI);
  }
  while (phi <= (-PI)) {
    phi += (2 * PI);
  }
  return phi;
}

float KalmanFilter::calculatePhi(float x, float y) {
  float phi = atan2(y, x);
  return phi;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd z_pred = calculateZPred(x_);
  VectorXd z_var = z;
  z_var[1] = normalizePhi(z[1]);
  VectorXd y = z_var - z_pred;
  y[1] = normalizePhi(y[1]);

  generateUpdate(y);
}

