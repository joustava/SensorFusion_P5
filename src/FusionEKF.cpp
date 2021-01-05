#include <iostream>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * @brief Construct a new Fusion E K F:: Fusion E K F object
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  noise_ax = 9;
  noise_ay = 9;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessInit(const MeasurementPackage &measurement_pack) {
    VectorXd x_in = VectorXd(4);
    previous_timestamp_ = measurement_pack.timestamp_;

    MatrixXd P_in = MatrixXd(4, 4);
    P_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
  
    MatrixXd F_in = MatrixXd(4, 4);
    F_in << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    MatrixXd Q_in = MatrixXd(4, 4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];

      x_in << rho * cos(phi), 
              rho * sin(phi), 
              0, 
              0;

      MatrixXd Hj = tools.CalculateJacobian(ekf_.x_);

      ekf_.Init(x_in, P_in, F_in, H_laser_, R_radar_, Q_in);
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x_in << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;
      
      ekf_.Init(x_in, P_in, F_in, H_laser_, R_radar_, Q_in);
    }
    is_initialized_ = true;
}

void FusionEKF::ProcessPrediction(const MeasurementPackage &measurement_pack) {
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict(dt);
}

void FusionEKF::ProcessUpdate(const MeasurementPackage &measurement_pack) {
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    ProcessInit(measurement_pack);
    return;
  }

  /**
   * Prediction
   */
  ProcessPrediction(measurement_pack);
  
  /**
   * Update
   */
  ProcessUpdate(measurement_pack);

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
