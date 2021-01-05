#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

void KalmanFilter::Predict(float delta_T) {
  F_(0, 2) = delta_T;
  F_(1, 3) = delta_T;

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  Estimate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd z_pred = tools.ConvertCartesianToPolar(x_);
  VectorXd y = z - z_pred;
    
  if (y(1) > M_PI) y(1) -= 2 * M_PI;
  if (y(1) < -M_PI) y(1) += 2 * M_PI;

  Estimate(y);
}

/**
 * PRIVATE
 */

MatrixXd KalmanFilter::Gain() {
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;
  return K;
}

void KalmanFilter::Estimate(const VectorXd &y) {
  MatrixXd K = Gain();
  int size = x_.size();
  
  MatrixXd I = MatrixXd::Identity(size, size);
  
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}
